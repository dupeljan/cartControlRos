#include <algorithm>
#include <boost/bind.hpp>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <vector>
#include <string>

#include <CartConrolPlugin/Velocity.h>
#include <CartConrolPlugin/Position.h>

#include <dynamic_reconfigure/server.h>
#include <CartConrolPlugin/CartConfig.h>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#define L 0.04 // distance between body center and wheel center
#define r 0.01905 // wheel radius
#define pi2 6.28318530717958623199592693708837032318115234375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) two * acos(mOne));

#define DEBUG 0 // send debug info to std::out
#define DYNAMIC_PID_RECONFIG_ENABLE 1 // Enable dynamic reconfig server for pid values


double normalizeRadian(double radian) {
    radian = fmod(radian, pi2);
    if (radian < 0) radian += pi2;
    return radian;
}

std::string positionToString(const CartConrolPlugin::Position p){
    return  "x " + std::to_string(p.x) +'\n' +
            "y " + std::to_string(p.y) + '\n' +
            "angle " + std::to_string(p.angle) + '\n';
}


namespace gazebo {

    class OmniPlatformPlugin : public ModelPlugin {
        // ros members
        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

        // end ros members

        private: physics::JointPtr backJoint;

        private: physics::JointPtr leftJoint;

        private: physics::JointPtr rightJoint;

          // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        private: physics::WorldPtr world;

        // Cart position publisher
        private: ros::Publisher positionPub;

        // Cart actual velocity publisher
        private: ros::Publisher velocityPub;

        // Thread of pos pulishing
        private: std::thread rosPosThread;

        // Rate var pointer for publishing delay
        private:  std::unique_ptr<ros::Rate> loop_rate;

        // Pid variable for wheel control
        private: common::PID pid;

        // Thread for dynamic reconf server
        private: std::thread dynamicReconfThread;

        // Server for dynamic retonfigure PID values
       // private: dynamic_reconfigure::Server<CartConrolPlugin::CartConfig> server;

        // Callback for dynamic reconfigure server
        //private: dynamic_reconfigure::Server<CartConrolPlugin::CartConfig>::CallbackType f;

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model
            this->model = _parent;

            this-> leftJoint = model->GetJoint("left_joint");
            this-> backJoint = model->GetJoint("back_joint");
            this->rightJoint = model->GetJoint("right_joint");

            this->world = _parent->GetWorld();

            // Setup pid
            this->pid = common::PID(3.0, 0.0, 0.0);
            // Apply the P-controller to the joints
            this->model->GetJointController()->SetVelocityPID(
                  this->leftJoint->GetScopedName(), this->pid);

            this->model->GetJointController()->SetVelocityPID(
                  this->rightJoint->GetScopedName(), this->pid);

            this->model->GetJointController()->SetVelocityPID(
                  this->backJoint->GetScopedName(), this->pid);

            


            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            //this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            //        boost::bind(&OmniPlatformPlugin::OnUpdate, this, _1));

            // Ros support part
            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
              int argc = 0;
              char **argv = NULL;
              ros::init(argc, argv, "gazebo_client",
                  ros::init_options::NoSigintHandler);
            }

            // Create our ROS node.
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            auto velocityTopicName = "/" + this->model->GetName() + "/velocity";
            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
              ros::SubscribeOptions::create<CartConrolPlugin::Velocity>(
                  velocityTopicName,
                  1,
                  boost::bind(&OmniPlatformPlugin::OnRosMsg, this, _1),
                  ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);
#if DEBUG == 1
            std::cout<<"Create topic "+ velocityTopicName +"\n";
#endif
            // Spin up the queue helper thread.
            this->rosQueueThread =
              std::thread(std::bind(&OmniPlatformPlugin::QueueThread, this));

            //
            //
            // Create publisher
            auto posPubTopicName = "/"+this->model->GetName() +"/pos";
            auto velocityPubTopicName = "/" + this->model->GetName() +"/actual_velocity";
            
            this->positionPub = this->rosNode->advertise<CartConrolPlugin::Position>(posPubTopicName, 1000);
            this->velocityPub = this->rosNode->advertise<CartConrolPlugin::Velocity>(velocityPubTopicName, 1000);
            // Set loop rate
            loop_rate = std::unique_ptr<ros::Rate>(new ros::Rate(10));
            // Run routine - public robot position

            this->rosPosThread =
                    std::thread(std::bind(&OmniPlatformPlugin::PublisherLoop,this));

            ///
            ///
            ///
            /// Create reconf server thread loop
#if DYNAMIC_PID_RECONFIG_ENABLE
            this->dynamicReconfThread =
                    std::thread(std::bind(&OmniPlatformPlugin::dynamicReconfLoop,this));
#endif
        }

        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        public: void OnRosMsg(const CartConrolPlugin::VelocityConstPtr &_msg)
        {
            // if stop - shitdown topic
            if (_msg->stop)
                this->rosNode->shutdown();

#if DEBUG == 1
            // Print velocityes
            std::cout<< "back: " + std::to_string(_msg->back) + '\n';
            std::cout<< "left: " + std::to_string(_msg->left) + '\n';
            std::cout<< "right: " + std::to_string(_msg->right) + '\n';
#endif
            // set velocityes
            leftJoint->SetVelocity(0, _msg->left / r);
            rightJoint->SetVelocity(0, _msg->right / r);
            backJoint->SetVelocity(0, _msg->back / r);
        }

        /// \brief ROS helper function that processes messages
        private: void QueueThread()
        {
          static const double timeout = 0.01;
          while (this->rosNode->ok())
          {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
          }
        }

    private: void PublisherLoop(){
            int count = 0;
            CartConrolPlugin::Position p;
            CartConrolPlugin::Velocity v;
            while (ros::ok())
             {

              // Get position
              p.x = this->model->RelativePose().Pos().X();
              p.y = this->model->RelativePose().Pos().Y();
              p.angle = this->model->RelativePose().Rot().Yaw();

              // Get velocity
              v.left = this->leftJoint->GetVelocity(0) * r;
              v.right = this->rightJoint->GetVelocity(0) * r  ;
              v.back = this->backJoint->GetVelocity(0) * r ;


#if DEBUG == 1
               std::cout << positionToString(p) << "------------\n";
#endif

               /**
                * The publish() function is how you send messages. The parameter
                * is the message object. The type of this object must agree with the type
                * given as a template parameter to the advertise<>() call, as was done
                * in the constructor above.
                */

               this->positionPub.publish(p);
               this->velocityPub.publish(v);

               ros::spinOnce();

               this->loop_rate->sleep();
               ++count;
             }
    }

    private: void dynamicReconfLoop()
    {
            dynamic_reconfigure::Server<CartConrolPlugin::CartConfig> server;
            dynamic_reconfigure::Server<CartConrolPlugin::CartConfig>::CallbackType f;

            f = boost::bind(&OmniPlatformPlugin::callbackDynamicReconf,this, _1, _2);

            server.setCallback(f);

            ROS_INFO("Spinning node");
            ros::spin();
    }

    void callbackDynamicReconf(CartConrolPlugin::CartConfig &config, uint32_t level) {
       ROS_INFO("Reconfigure Request: %d %f %s %s %d",
                config.int_param, config.double_param,
                config.str_param.c_str(),
                config.bool_param?"True":"False",
                config.size);
    }



    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(OmniPlatformPlugin)
}


