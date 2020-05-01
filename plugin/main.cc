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

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#define L 0.04 // distance between body center and wheel center
#define r 0.01905 // wheel radius






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



        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model
            this->model = _parent;

            this-> leftJoint = model->GetJoint("left_joint");
            this-> backJoint = model->GetJoint("back_joint");
            this->rightJoint = model->GetJoint("right_joint");

            this->world = _parent->GetWorld();
            //this->indicator = this->world->GetModel("wood_cube_2_5cm");
            //this->indicatorPose.reset(new math::Pose());
            //this->indicator->SetGravityMode(false);
            //this->indicator->Fini();

            std::srand(std::time(0));



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

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
              ros::SubscribeOptions::create<std_msgs::Float32>(
                  "/" + this->model->GetName() + "/vel_cmd",
                  1,
                  boost::bind(&OmniPlatformPlugin::OnRosMsg, this, _1),
                  ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
              std::thread(std::bind(&OmniPlatformPlugin::QueueThread, this));
        }

        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
        {
            leftJoint->SetVelocity(0, _msg->data / r);
            if(_msg->data == 11.0)
                this->rosNode->shutdown();
            //rightJoint->SetVelocity(0, _msg->data / r);
            //backJoint->SetVelocity(0, _msg->data / r);
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



    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(OmniPlatformPlugin)
}


