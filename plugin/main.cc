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
#include <map>
#include <mutex>
#include <string>
#include <ctime>




#include "kinematic/cartkinematic.h"

#include <CartConrolPlugin/PathSrv.h>

#include <CartConrolPlugin/VelocityCart.h>
#include <CartConrolPlugin/VelocityWheels.h>
#include <CartConrolPlugin/Position.h>
#include <CartConrolPlugin/PathMsg.h>

#include <dynamic_reconfigure/server.h>
#include <CartConrolPlugin/CartConfig.h>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"

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
        ///  A node use for ROS transport for publication
        private: std::unique_ptr<ros::NodeHandle> rosNodePub;

        /// A node use for ROS transport for publication
        private: std::unique_ptr<ros::NodeHandle> rosNodeSub;

        /// A ROS subscriber
        private: ros::Subscriber rosSub;

        /// A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;

        ///  A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

        /// map of subsc. options for subsciber manipulating
        private: std::map<std::string,ros::SubscribeOptions> soMap;


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

        // Publisher for reveseKinematic
        private: ros::Publisher reverseKinematicPub;

        // Difference between actial speed and required
        //private: ros::Publisher velocidyDiffPub;

        // Thread of pos pulishing
        private: std::thread rosPosThread;

        // Rate var pointer for publishing delay
        private:  std::unique_ptr<ros::Rate> loop_rate;

        // Pid variable for wheel control
        private: common::PID pidWheels;

        // Pid varieble for cart rotation
        private: common::PID pidCartRot;

        // Pid variable for right path stearing
        private: common::PID pidStearing;

        // Time for rotation pid
        private: std::clock_t time;

        private: common::Time simTime;

        private: ros::Time rosTime;

        // Thread for dynamic reconf server
        private: std::thread dynamicReconfThread;

        // Thread for path service
        private: std::thread pathSrvThread;

        // Stearing error for path correcting
        private: double stearingErr;

        // Stearing direction
        private: CartConrolPlugin::VelocityWheels vDesiredWPerp;



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
            this->pidWheels = common::PID(5e-4, 0.0, 0.0);
            // Apply the P-controller to the joints
            this->applyPID();

            this->pidCartRot = common::PID(50.0,25.0,6.0);

            this->pidStearing = common::PID(1.0,1.0);

            this->time = std::clock();

            this-> simTime = this->world->SimTime();

            this->rosTime = ros::Time::now();





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
              ros::init(argc, argv, "gazebo_cart_plugin",
                  ros::init_options::NoSigintHandler);
            }

            // Create our ROS nodes
            this->rosNodeSub.reset(new ros::NodeHandle("gazebo_cart_plugin_sub"));
            this->rosNodePub.reset(new ros::NodeHandle("gazebo_cart_plugin_pub"));

             // Set param for sinchronize
            // Doesn't work
            //this->rosNodeSub->setParam("use_sim_time",true);

            auto velocityTopicName = "/" + this->model->GetName() + "/velocity";
            auto pathTopicName = "/" + this->model->GetName() + "/path";
            // Create a named topic, and subscribe to it.
            this->soMap["Velocity"]
                    =  ros::SubscribeOptions::create<CartConrolPlugin::VelocityWheels>(
                            velocityTopicName,
                            1,
                            boost::bind(&OmniPlatformPlugin::OnRosMsgVel, this, _1),
                            ros::VoidPtr(), &this->rosQueue);


            this->subscribe("Velocity");

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
            auto velocityPubTopicName = "/" + this->model->GetName() + "/actual_velocity";
            auto reverseKinematic = "/" + this->model->GetName() + "/reverse_kinematic";

            this->positionPub = this->rosNodePub->advertise<CartConrolPlugin::Position>(posPubTopicName, 2000);
            this->velocityPub = this->rosNodePub->advertise<CartConrolPlugin::VelocityWheels>(velocityPubTopicName, 2000);
            this->reverseKinematicPub = this->rosNodePub->advertise<CartConrolPlugin::VelocityCart>(reverseKinematic, 2000);
            // Set loop rate
            loop_rate = std::unique_ptr<ros::Rate>(new ros::Rate(20000));
            // Run routine - public robot position

            this->rosPosThread =
                    std::thread(std::bind(&OmniPlatformPlugin::PublisherLoop,this));

            // Create path server
            this->pathSrvThread =
                    std::thread(std::bind(&OmniPlatformPlugin::pathServerRoutine,this));
            ///
            ///
            ///
            /// Create reconf server thread loop
#if DYNAMIC_PID_RECONFIG_ENABLE == 1
            this->dynamicReconfThread =
                    std::thread(std::bind(&OmniPlatformPlugin::dynamicReconfLoop,this));
#endif
        }

        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        public: void OnRosMsgVel(const CartConrolPlugin::VelocityWheelsConstPtr &_msg)
        {
            /*
            // if stop - shutdown topic and start path stearing
            if (_msg->stop)
            {
                this->rosSub.shutdown();
                this->subscribe("PathMsg");
            }

            */

#if DEBUG == 1
            // Print velocityes
            std::cout<< "back: " + std::to_string(_msg->back) + '\n';
            std::cout<< "left: " + std::to_string(_msg->left) + '\n';
            std::cout<< "right: " + std::to_string(_msg->right) + '\n';
#endif



            // set velocityes
            setTargetVelocity(_msg->left,_msg->right,_msg->back);



            //this->model->GetJointController()->Update();
           // leftJoint->SetVelocityTarget(0, _msg->left / r);
           // rightJoint->SetVelocityTarget(0, _msg->right / r);
           // backJoint->SetVelocityTarget(0, _msg->back / r);
        }

    public:  bool onPathGet(CartConrolPlugin::PathSrv::Request &req,
                            CartConrolPlugin::PathSrv::Response &res)
        {
            for( auto x : req.path)
                ROS_INFO("%f %f",x.position.x,x.position.y);
            this->rosSub.shutdown();
            auto th =
                    std::thread(std::bind(&OmniPlatformPlugin::OnRosMsgPath,this,req.path));
            th.detach();
            //OnRosMsgPath(req.path);
            res.status = true;
            return true;
        }
    private: void OnRosMsgPath(const std::vector<geometry_msgs::Pose> _msg)
        {
#if DEBUG == -1
            for (int i=0; i<_msg->path.size(); ++i)
               {
                 auto &data = _msg->path[i];
                 ROS_INFO("X: %f Y: %f" ,
                          data.position.x,
                          data.position.y );
               }
#endif
            /// Send message to client
            /// that we already receved
            /// path



            // Transform geometry position to
            // CartKinematic::PointF
            std::vector<CartKinematic::PointF> points;
            // Get current pos
            {
                CartKinematic::PointF p;
                p.x = this->model->RelativePose().Pos().X();
                p.y = this->model->RelativePose().Pos().Y();
                points.push_back(p);
            }
            // Transform path to points
            std::transform(_msg.begin(),_msg.end(),
                           std::back_insert_iterator<std::vector<CartKinematic::PointF>>(points),
                           []( geometry_msgs::Pose pose){
                                CartKinematic::PointF point;
                                point.x = pose.position.x;
                                point.y = pose.position.y;
                                return point;
            });

            int freq = 1000;
            // Velocity unit per second
            double velocity = 0.1;
            auto rate = ros::Rate(freq);
            // Reset time
            this->rosTime = ros::Time::now();

            // Steer cart the way
            for(auto it = points.begin(); (it + 1) != points.end(); it++)
            {
                // it[0] - start point
                // it[1] - destination point
                // Get distance between points
                auto d = CartKinematic::distance(it[0],it[1]);
                // Get cart velocity vector
                auto tau = it[1] - it[0];
                CartConrolPlugin::VelocityCart vDesiredC;
                vDesiredC.x = tau.x;
                vDesiredC.y = tau.y;
                vDesiredC.angle = 0.0;
                auto vDesiredCNorm = CartKinematic::norm(vDesiredC);
                // Get perpendicular to desired movement
                CartKinematic::PointF vDesiredCPrep;
                auto sign = (std::signbit(tau.x*tau.y))? -1 : 1;
                vDesiredCPrep.x = sign * tau.y;
                vDesiredCPrep.y = - sign * tau.y;
                this->vDesiredWPerp =
                        CartKinematic::getVelocity(vDesiredCPrep);

                // Get wheel velocity vector
                CartConrolPlugin::VelocityWheels vDesiredW
                        = CartKinematic::getVelocity(tau);
                // Actual cart velosity
                CartConrolPlugin::VelocityWheels vActW;
                std::cout<< "d " <<std::to_string(d) << std::endl;
                // Move robot
                // Distance treveled
                double s = 0.0;
                // Error vector len
                this->stearingErr  = 0.0;

                CartKinematic::PointF before, after,b, a;
                before.x = this->model->RelativePose().Pos().X();
                before.y = this->model->RelativePose().Pos().Y();
                b = before;
                
                while ( s < d )
                {
                    this->setTargetVelocity(vDesiredW.left,vDesiredW.right,vDesiredW.back,true);
                    // Add treveled distance

                    vActW.left = this->leftJoint->GetVelocity(0);
                    vActW.right =  this->rightJoint->GetVelocity(0);
                    vActW.back =  this->backJoint->GetVelocity(0);
                    auto vActC = CartKinematic::getVelocityReverse(vActW);
                    vActC.angle = 0;
                    //// vvv convertation to unit/sec
                    // Translate vAct on v direction and oposite v direction
                    // Get cos angle between vActC and vDesiredC
                    double vActCNorm =
                            CartKinematic::norm(vActC);
                    double cosAngle
                            = CartKinematic::scalarMul(vActC,vDesiredC) /
                            (vActCNorm * vDesiredCNorm);
                    double vStearing = cosAngle * vActCNorm;
                    this->stearingErr = std::sqrt( 1 - std::pow( cosAngle, 2 ) ) * vActCNorm;
                    //std::cout << "Stearing " << vStearing << " error " << vError << " cos " << cosAngle <<'\n';
                    //s += 0.2 * CartKinematic::norm(vActC) / freq;
                    s += 0.14 * 7.07 * vStearing / freq;
                    // cheat
                    //a.x = this->model->RelativePose().Pos().X();
                    //a.y = this->model->RelativePose().Pos().Y();
                    //s += CartKinematic::distance(b,a);
                    //b = a;
                    // Sleep for next iteration

                    rate.sleep();
                }
                after.x = this->model->RelativePose().Pos().X();
                after.y = this->model->RelativePose().Pos().Y();
                std::cout << "actual d " << CartKinematic::distance(before,after) << '\n';
                std::cout << "s " << s << '\n';
            }


            auto eps = 0.1;
            // Stop cart
            while (this->pidCartRot.GetCmd() > eps ||

                   this->leftJoint->GetVelocity(0) > eps ||
                   this->rightJoint->GetVelocity(0) > eps ||
                   this->backJoint->GetVelocity(0) > eps ||

                   this->pidWheels.GetCmd() > std::pow(eps,2)
       ){
                this->setTargetVelocity(0.0,0.0,0.0);
                rate.sleep();
            }

            std::cout << "Finish path steering\n";


            // Start velocity control
            //this->rosSub.shutdown();
            this->subscribe("Velocity");
        }

        /// \brief ROS helper function that processes messages
        private: void QueueThread()
        {
          static const double timeout = 0.01;
          while (this->rosNodeSub->ok())
          {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
          }
        }
        // Set target velocity to the cart
    private: void setTargetVelocity(double left, double right, double back,bool isOnPath = false)
        {

            // Add stearing impact
            if (isOnPath){
                auto duration =  (ros::Time::now().toNSec() - rosTime.toNSec()) / 1e9;
                this->rosTime = ros::Time::now();
                auto errShift = this->pidStearing.Update(this->stearingErr,common::Time(duration));

                left += errShift * this->vDesiredWPerp.left;
                right += errShift * this->vDesiredWPerp.right;
                back += errShift * this->vDesiredWPerp.back;
            }

            // Add rotation impact
            // Rotation correction
            auto err =   this->model->RelativePose().Rot().Yaw();
            //std::cout << duration<< "    cTIme-> " << 1000.0 * (std::clock()-time)/ CLOCKS_PER_SEC << std::endl;
            auto rc = pidCartRot.Update(err,common::Time( 1000.0 * (std::clock()-time)/ CLOCKS_PER_SEC) );
            this->time = std::clock();
            this->simTime = this->world->SimTime();
#if DEBUG == 1
            std::cout << "Error:" << std::to_string(err) <<" Corrective speed:" << std::to_string(rc) << '\n';
#endif
            this->model->GetJointController()->
                    SetVelocityTarget( this->leftJoint->GetScopedName(), rc + left);
            this->model->GetJointController()->
                    SetVelocityTarget( this->rightJoint->GetScopedName(), rc + right);
            this->model->GetJointController()->
                    SetVelocityTarget( this->backJoint->GetScopedName(), rc + back);
        }

    private: void PublisherLoop(){
            int count = 0;
            CartConrolPlugin::Position p;
            CartConrolPlugin::VelocityCart reverseKinematic;
            CartConrolPlugin::VelocityWheels v;
            //CartConrolPlugin::Velocity vDiff;
            while (ros::ok())
             {

             // std::cout << this->world->SimTime().Double() <<std::endl;
              // Get position
              p.x = this->model->RelativePose().Pos().X();
              p.y = this->model->RelativePose().Pos().Y();
              p.angle = this->model->RelativePose().Rot().Yaw();

              /*
              auto getVel =  this->model->GetJointController()->GetVelocities();
              v.left = getVel["robot_mobile_wheel_3_omni_open_base::left_joint"];
              v.right = getVel["robot_mobile_wheel_3_omni_open_base::right_joint"];
              v.back = getVel["robot_mobile_wheel_3_omni_open_base::back_joint"];
              for(auto it = getVel.begin(); it != getVel.end(); it++ )
                  std::cout << it->first  // string (key)
                               << ':'
                               << it->second   // string's value
                               << std::endl ;
               */
              // Get velocity
             v.left = this->leftJoint->GetVelocity(0); //* r;
             v.right = this->rightJoint->GetVelocity(0); //* r  ;
             v.back = this->backJoint->GetVelocity(0); //* r ;


             // get Reverse kinematic
             reverseKinematic = CartKinematic::getVelocityReverse(v);
              // Get velocity diffenence
              //vDiff =

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
               this->reverseKinematicPub.publish(reverseKinematic);

               //ros::spinOnce();
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
            //ros::spin();
    }

    private: void pathServerRoutine()
    {
            ros::NodeHandle n;
            auto f = boost::bind(&OmniPlatformPlugin::onPathGet,this,_1,_2);
            ros::ServiceServer service = n.advertiseService<CartConrolPlugin::PathSrv::Request,
                                      CartConrolPlugin::PathSrv::Response>("pathSrv", f);
            ROS_INFO("Start service pathSrv");
            ros::MultiThreadedSpinner spinner(2);
            spinner.spin();
            //ros::spin();
    }
    private: void callbackDynamicReconf(CartConrolPlugin::CartConfig &config, uint32_t level) {
       ROS_INFO("Reconfigure Request: P - %f I - %f D - %f level %d",
                config.p_gain,
                config.i_gain,
                config.d_gain,
                level
             );
       if (!level)
       {
           this->pidWheels.SetPGain(config.p_gain);
           this->pidWheels.SetIGain(config.i_gain);
           this->pidWheels.SetDGain(config.d_gain);
           applyPID();
       }
       else
       {
           this->pidCartRot.SetPGain(config.prc_gain);
           this->pidCartRot.SetIGain(config.irc_gain);
           this->pidCartRot.SetDGain(config.drc_gain);
       }


    }

    private: void applyPID()
    {
        // Apply the P-controller to the joints
        this->model->GetJointController()->SetVelocityPID(
              this->leftJoint->GetScopedName(), this->pidWheels);

        this->model->GetJointController()->SetVelocityPID(
              this->rightJoint->GetScopedName(), this->pidWheels);

        this->model->GetJointController()->SetVelocityPID(
              this->backJoint->GetScopedName(), this->pidWheels);
    }

        /// subscribe tocurrent soStruct topic
        private: void subscribe(std::string key)
        {
            auto so = this->soMap[key];
            this->rosSub = this->rosNodeSub->subscribe(so);
        }
    };



    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(OmniPlatformPlugin)
}


