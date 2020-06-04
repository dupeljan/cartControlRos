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
#define pi 3.1415926535897932
#define DEBUG 0 // send debug info to std::out
#define DYNAMIC_PID_RECONFIG_ENABLE 1 // Enable dynamic reconfig server for pid values
#define STATISTIC_ENABLE 1 // Enable counting stearing error statistic

/*
 * return radiand in [-2pi,2pi] limits
 * @ radian - input radian value
 * return - radians, mapped to [-2pi,2pi]
*/
double normalizeRadian(double radian) {
    radian = fmod(radian, pi2);
    if (radian < 0) radian += pi2;
    return radian;
}

/*
 * Translate posittion to readable string
 * @ p - input point
 * return - readable string
*/
std::string positionToString(const CartControlPlugin::Position p){
    return  "x " + std::to_string(p.x) +'\n' +
            "y " + std::to_string(p.y) + '\n' +
            "angle " + std::to_string(p.angle) + '\n';
}


namespace gazebo {

    /*
     * Custom Gazebo model plugin implementation
     * Allow to control cart by
     * continious /velocity message
     * and pathSrv service
    */
    class OmniPlatformPlugin : public ModelPlugin {

        // ros members
        //  Node use for ROS transport for publication
        private: std::unique_ptr<ros::NodeHandle> rosNodePub;

        // Node use for ROS transport for publication
        private: std::unique_ptr<ros::NodeHandle> rosNodeSub;

        // ROS subscriber
        private: ros::Subscriber rosSub;

        // ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;

        //  Thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

        // Map of subsc. options for subsciber manipulating
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

        // Time for pid control
        private: ros::Time rosTime;

        // Thread for dynamic reconf server
        private: std::thread dynamicReconfThread;

        // Thread for path service
        private: std::thread pathSrvThread;

        // Stearing error for path correcting
        private: double stearingErr;

        // Stearing direction
        private: CartControlPlugin::VelocityWheels vDesiredWPerp;


    ///
    ///
    /// MAIN
    /// point of entry
    ///
    ///
    /*
     * Calls on load of model in simulation
     * @ _parent - cart model pointer
     * @ _sdf - don't know
    */
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store model values
        this->model = _parent;

        this-> leftJoint = model->GetJoint("left_joint");
        this-> backJoint = model->GetJoint("back_joint");
        this->rightJoint = model->GetJoint("right_joint");

        this->world = _parent->GetWorld();

        // Setup pid
        this->pidWheels = common::PID(5e-4, 0.0, 0.0);

        this->applyPID();

        this->pidCartRot = common::PID(50.0,25.0,6.0);

        this->pidStearing = common::PID(6.0,3.0);

         // Setup time
        this->rosTime = ros::Time::now();

        // Init ros
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_cart_plugin",
            ros::init_options::NoSigintHandler);
        }

        // Create ROS nodes
        this->rosNodeSub.reset(new ros::NodeHandle("gazebo_cart_plugin_sub"));
        this->rosNodePub.reset(new ros::NodeHandle("gazebo_cart_plugin_pub"));

        // Set param for sinchronize
        // Doesn't work
        //this->rosNodeSub->setParam("use_sim_time",true);

        auto velocityTopicName = "/" + this->model->GetName() + "/velocity";
        // Create a named topic, and subscribe to it.
        this->soMap["Velocity"]
        =  ros::SubscribeOptions::create<CartControlPlugin::VelocityWheels>(
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

        // Create publishers
        auto posPubTopicName = "/"+this->model->GetName() +"/pos";
        auto velocityPubTopicName = "/" + this->model->GetName() + "/actual_velocity";
        auto reverseKinematic = "/" + this->model->GetName() + "/reverse_kinematic";

        this->positionPub = this->rosNodePub->advertise<CartControlPlugin::Position>(posPubTopicName, 2000);
        this->velocityPub = this->rosNodePub->advertise<CartControlPlugin::VelocityWheels>(velocityPubTopicName, 2000);
        this->reverseKinematicPub = this->rosNodePub->advertise<CartControlPlugin::VelocityCart>(reverseKinematic, 2000);

        // Setup loop rate
        loop_rate = std::unique_ptr<ros::Rate>(new ros::Rate(20000));

        // Run publication thread loop
        this->rosPosThread =
            std::thread(std::bind(&OmniPlatformPlugin::PublisherLoop,this));

        // Run path getter service thread loop
        this->pathSrvThread =
            std::thread(std::bind(&OmniPlatformPlugin::pathServerRoutine,this));

        // Create reconf service thread loop
        #if DYNAMIC_PID_RECONFIG_ENABLE == 1
            this->dynamicReconfThread =
                std::thread(std::bind(&OmniPlatformPlugin::dynamicReconfLoop,this));
        #endif
    }

        /*
         * Velocity subscriber callback
         * @ _msg - consist wheels speed values
        */
    public: void OnRosMsgVel(const CartControlPlugin::VelocityWheelsConstPtr &_msg)
    {
        #if DEBUG == 1
            // Print velocityes
            std::cout<< "back: " + std::to_string(_msg->back) + '\n';
            std::cout<< "left: " + std::to_string(_msg->left) + '\n';
            std::cout<< "right: " + std::to_string(_msg->right) + '\n';
        #endif

        // Set target velocityes
       this->setTargetVelocity(_msg->left,_msg->right,_msg->back);

    }

        /*
         * Path getter service callback
         * @ req - request, consist the path
         * @ res - responce, consist respose status
        */
    public:  bool onPathGet(CartControlPlugin::PathSrv::Request &req,
                            CartControlPlugin::PathSrv::Response &res)
    {
        // Log recived path
        for( auto x : req.path)
            ROS_INFO("%f %f",x.position.x,x.position.y);

        // Stop follow the /velocity topic
        this->rosSub.shutdown();

        // Start path following
        auto th =
            std::thread(std::bind(&OmniPlatformPlugin::OnRosMsgPath,this,req.path));
        th.detach();

        // Return status true
        res.status = true;
        return true;
    }

        /*
         * Force cart follow the path
         * from current positon
         * @ _msg - path description. Consist of vector of points
        */
    private: void OnRosMsgPath(const std::vector<geometry_msgs::Pose> _msg)
    {
        #if DEBUG == -1
        //  Log out recived path
        for (int i=0; i<_msg->path.size(); ++i)
        {
            auto &data = _msg->path[i];
            ROS_INFO("X: %f Y: %f" ,
            data.position.x,
            data.position.y );
        }
        #endif


        // Reverse kinematic pos
        CartControlPlugin::Position posRK;
        // Transform geometry position to
        // CartKinematic::PointF
        std::vector<CartKinematic::PointF> points;
        // Get current pos
        {
            CartKinematic::PointF p;
            p.x = this->model->RelativePose().Pos().X();
            p.y = this->model->RelativePose().Pos().Y();
            points.push_back(p);

            posRK.x = p.x;
            posRK.y = p.y;
        }

        // Transform path to points vector
        std::transform(_msg.begin(),_msg.end(),
        std::back_insert_iterator<std::vector<CartKinematic::PointF>>(points),
        []( geometry_msgs::Pose pose)
        {
            CartKinematic::PointF point;
            point.x = pose.position.x;
            point.y = pose.position.y;
            return point;
        });

        // Update frequence Hz
        int freq = 1000;
        // Coef to manipulate turning distance
        double turnSrink = 0.04;
        // Rate for time manipulating
        auto rate = ros::Rate(freq);
        // Reset time
        this->rosTime = ros::Time::now();

        #if STATISTIC_ENABLE == 1
            // Statistic
            double maxError = 0.0;
            double sumError = 0.0;
            long long count = 0;
        #endif

        // Steer cart the way
        for(auto it = points.begin(); (it + 1) != points.end(); it++)
        {
            // it[0] - start point
            // it[1] - destination point

            // Get distance between points
            auto d = CartKinematic::distance(it[0],it[1]);

            // Get cart velocity vector
            auto tau = it[1] - it[0];

            // Desired velocity in cart frame
            CartControlPlugin::VelocityCart vDesiredC;
            vDesiredC.x = tau.x;
            vDesiredC.y = tau.y;
            vDesiredC.angle = 0.0;

            // Length of the desired velocity in cart frame
            auto vDesiredCNorm = CartKinematic::norm(vDesiredC);

            // Get perpendicular to desired velocity vector in cart frame
            CartKinematic::PointF vDesiredCPrep;
            auto sign = (std::signbit(tau.x*tau.y))? -1 : 1;
            sign *= (std::signbit(tau.y))? -1: 1;
            vDesiredCPrep.x = -sign * tau.y;
            vDesiredCPrep.y = sign * tau.x;

            // Perpedicular of desired velocity in wheels frame
            this->vDesiredWPerp =
            CartKinematic::getVelocity(vDesiredCPrep);

            // Get desired velocity vector in wheels frame
            CartControlPlugin::VelocityWheels vDesiredW
                = CartKinematic::getVelocity(tau);
            // Actual cart velosity vector in wheels frame
            CartControlPlugin::VelocityWheels vActW;
            #if DEBUG == 1
                std::cout<< "Segment len " << d << std::endl;
            #endif

            // Distance treveled
            double s = 0.0;

            // Error vector len
            this->stearingErr  = 0.0;

            // Coords to compare real distance
            // and reverse kinematic distance
            CartKinematic::PointF before, after,b, a;
            before.x = this->model->RelativePose().Pos().X();
            before.y = this->model->RelativePose().Pos().Y();
            b = before;

            // Compute place where cart
            // must turning
            double dDesired = d;
            // If there are >= 3 points in the path
            if( (it + 2) != points.end())
            {
                // Get next turning angle
                auto angle =
                    CartKinematic::angleOpositeBC(it[1],it[0],it[2]);
                #if DEBUG == 1
                    std::cout << "Next turn angle " << angle << std::endl;
                #endif

                // Shrink moving length
                dDesired -= turnSrink * (pi -  std::abs(angle));

            }
            // Move cart
            // While current path length < desired path length
            while ( s < dDesired )
            {
                // Set target speed
                this->setTargetVelocity(vDesiredW.left,vDesiredW.right,vDesiredW.back,true);

                // Get actual wheel speed
                vActW.left = this->leftJoint->GetVelocity(0);
                vActW.right =  this->rightJoint->GetVelocity(0);
                vActW.back =  this->backJoint->GetVelocity(0);

                // Get actual speed vector in cart frame
                auto vActC = CartKinematic::getVelocityReverse(vActW);
                vActC.angle = 0;


                // Translate vAct on v direction and oposite v direction
                // Get cos angle between vActC and vDesiredC
                double vActCNorm = CartKinematic::norm(vActC);
                double cosAngle
                    = CartKinematic::scalarMul(vActC,vDesiredC) /
                        (vActCNorm * vDesiredCNorm);

                // Get path length only in desired direction
                double vStearing = cosAngle * vActCNorm;

                // Compute length between
                // current robot pos in posRK
                // and path segment line
                {
                    auto sign = (it[0].y > it[1].y)? 1 : -1;
                    sign *= (it[0].x <= it[1].x && it[0].y >= it[1].y ||
                    it[0].x >= it[1].x && it[0].y <= it[1].y)? -1 : 1;
                    this->stearingErr =
                        sign * CartKinematic::signDistanceToLine(it[0],it[1],CartKinematic::PointF(posRK.x,posRK.y));

                }

                // Add traveled distance
                //   vvv convertation to unit/sec
                s += 0.14 * 7.07 * vStearing / freq;

                // Cheat
                // Using real distanse instead of reverse kinematic one
                /*
                a.x = this->model->RelativePose().Pos().X();
                a.y = this->model->RelativePose().Pos().Y();
                s += CartKinematic::distance(b,a);
                b = a;
                */

                // Compute new pos
                posRK.x += 0.14 * 7.07 * vActC.x /(double) freq;
                posRK.y += 0.14 * 7.07 * vActC.y /(double) freq;

                // Cheat
                // Using real distanse instead of reverse kinematic one
                /*
                posRK.x = this->model->RelativePose().Pos().X();
                posRK.y = this->model->RelativePose().Pos().Y();
                */



                #if STATISTIC_ENABLE == 1
                    // Compute statistic
                    // Get distance between
                    // path and real cart pos
                    auto dst
                        = CartKinematic::signDistanceToLine(
                            it[0],it[1],
                            CartKinematic::PointF(this->model->RelativePose().Pos().X(),
                                this->model->RelativePose().Pos().Y()));
                    dst = std::abs(dst);
                    maxError = std::max(maxError, dst);
                    sumError += dst;
                    count++;
                #endif


                #if DEBUG == 1
                    std::cout << "Distance error " << CartKinematic::distance(
                      CartKinematic::PointF(posRK.x,posRK.y),
                        CartKinematic::PointF(this->model->RelativePose().Pos().X(),
                                              this->model->RelativePose().Pos().Y())) << std::endl;
                    */
                    // std::cout << "POS rk " << posRK.x << " " << posRK.y << std::endl;
                    // std::cout << "POS real " <<this->model->RelativePose().Pos().X()<< " " << this->model->RelativePose().Pos().Y() << std::endl;
                #endif

                // Sleep till next iteration
                rate.sleep();
            }

            #if DEBUG == 0
                // Log actual traveled distance
                // and computed traveled distance
                after.x = this->model->RelativePose().Pos().X();
                after.y = this->model->RelativePose().Pos().Y();
                std::cout << "actual d " << CartKinematic::distance(before,after) << '\n';
                std::cout << "s " << s << '\n';
            #endif
        }


        auto eps = 0.1;
        // Stop cart
        while
        (
            this->pidCartRot.GetCmd() > eps ||
            this->leftJoint->GetVelocity(0) > eps ||
            this->rightJoint->GetVelocity(0) > eps ||
            this->backJoint->GetVelocity(0) > eps ||
            this->pidWheels.GetCmd() > std::pow(eps,2)
        )
        {
            this->setTargetVelocity(0.0,0.0,0.0);
            rate.sleep();
        }

        std::cout << "Finish path steering" << std::endl;

        #if STATISTIC_ENABLE == 1
            std::cout << "Statistic:" << std::endl;
            std::cout << "Max error: " << maxError << std::endl;
            std::cout << "Average error: " << (sumError/(double) count) << std::endl;
        #endif

        // Start velocity control again
        this->subscribe("Velocity");
    }

        /*
         * Queue ros messages
        */
    private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNodeSub->ok())
            this->rosQueue.callAvailable(ros::WallDuration(timeout));

    }

        /*
         * Set target curt velocity
         * @ left
         * @ right
         * @ back -- target speed for correspond wheel
         * @ isOnPat activate PID for
         * steering error compensation
        */
    private: void setTargetVelocity(double left, double right, double back,bool isOnPath = false)
    {
        // Get duration
        auto duration =  (ros::Time::now().toNSec() - rosTime.toNSec()) / 1e9;

        // Update time
        this->rosTime = ros::Time::now();

        if (isOnPath)
        {
            // Get PID signal
            auto errShift = this->pidStearing.Update(this->stearingErr,common::Time(duration));

            // Add steering impact
            left += errShift * this->vDesiredWPerp.left;
            right += errShift * this->vDesiredWPerp.right;
            back += errShift * this->vDesiredWPerp.back;
        }


        // Get rotation error
        auto err =   this->model->RelativePose().Rot().Yaw();

        // Compute rotation PID signal
        auto rc = pidCartRot.Update(err,common::Time( 4000.0 * duration ));


        #if DEBUG == 1
            std::cout << "Error:" << std::to_string(err) <<" Corrective speed:" << std::to_string(rc) << '\n';
        #endif

        // Set target joints velocity
        this->model->GetJointController()->
            SetVelocityTarget( this->leftJoint->GetScopedName(), rc + left);
        this->model->GetJointController()->
            SetVelocityTarget( this->rightJoint->GetScopedName(), rc + right);
        this->model->GetJointController()->
            SetVelocityTarget( this->backJoint->GetScopedName(), rc + back);

    }

        /*
         * Publicate model values to topics
        */
    private: void PublisherLoop()
    {

        CartControlPlugin::Position p;
        CartControlPlugin::VelocityCart reverseKinematic;
        CartControlPlugin::VelocityWheels v;

        // While plugin is active
        while (ros::ok())
        {

            // Get real robot position
            p.x = this->model->RelativePose().Pos().X();
            p.y = this->model->RelativePose().Pos().Y();
            p.angle = this->model->RelativePose().Rot().Yaw();


            // Get velocity
            v.left = this->leftJoint->GetVelocity(0);
            v.right = this->rightJoint->GetVelocity(0);
            v.back = this->backJoint->GetVelocity(0);

            // Get Reverse kinematic
            reverseKinematic = CartKinematic::getVelocityReverse(v);

            #if DEBUG == 1
                std::cout << positionToString(p) << "------------\n";
            #endif


            // Send values to topics
            this->positionPub.publish(p);
            this->velocityPub.publish(v);
            this->reverseKinematicPub.publish(reverseKinematic);

            //ros::spinOnce();
            // Spin delay
            this->loop_rate->sleep();

        }
    }

        /*
         * Dynamic reconfigure main loop
        */
    private: void dynamicReconfLoop()
    {
        // Setup service
        ros::NodeHandle n("PID_reconfigure");
        dynamic_reconfigure::Server<CartControlPlugin::CartConfig> server(n);
        dynamic_reconfigure::Server<CartControlPlugin::CartConfig>::CallbackType f;

        f = boost::bind(&OmniPlatformPlugin::callbackDynamicReconf,this, _1, _2);

        // Setup callback
        server.setCallback(f);


        // Spin service
        ROS_INFO("Spinning  dynamic reconfig node");
        ros::MultiThreadedSpinner spinner(3);
        spinner.spin();
    }

        /*
         * Path getter service
         *  main loop
        */
   private: void pathServerRoutine()
   {
        // Setup callback
        ros::NodeHandle n;
        auto f = boost::bind(&OmniPlatformPlugin::onPathGet,this,_1,_2);
        ros::ServiceServer service = n.advertiseService<CartControlPlugin::PathSrv::Request,
                              CartControlPlugin::PathSrv::Response>("pathSrv", f);
        // Spin service
        ROS_INFO("Start service pathSrv");
        ros::MultiThreadedSpinner spinner(3);
        spinner.spin();
    }


        /*
         * Dynamic reconfigure main loop
         * @ &config consist reconfig info
         * @ level consist request level
        */
    private: void callbackDynamicReconf(CartControlPlugin::CartConfig &config, uint32_t level)
    {

       // Show reconfigure request
       ROS_INFO("Reconfigure Request: P - %f I - %f D - %f level %d",
                config.p_gain,
                config.i_gain,
                config.d_gain,
                level
             );
       // level == 0
       // means whells pid reconfiguration
       // level == 1
       // means rotation pid reconfiguration
       // level == 2
       // means steering pid reconfiguration
       if ( level == 0 )
       {
           this->pidWheels.SetPGain(config.p_gain);
           this->pidWheels.SetIGain(config.i_gain);
           this->pidWheels.SetDGain(config.d_gain);
           applyPID();
       }
       else if( level == 1 )
       {
           this->pidCartRot.SetPGain(config.prc_gain);
           this->pidCartRot.SetIGain(config.irc_gain);
           this->pidCartRot.SetDGain(config.drc_gain);
       }
       else if( level == 2 )
       {
           this->pidStearing.SetPGain(config.pst_gain);
           this->pidStearing.SetIGain(config.ist_gain);
           this->pidStearing.SetDGain(config.dst_gain);
       }

    }

        /*
         * Apply pid to wheel joints
        */
    private: void applyPID()
    {
        // Apply the PID-controller to the joints
        this->model->GetJointController()->SetVelocityPID(
              this->leftJoint->GetScopedName(), this->pidWheels);

        this->model->GetJointController()->SetVelocityPID(
              this->rightJoint->GetScopedName(), this->pidWheels);

        this->model->GetJointController()->SetVelocityPID(
              this->backJoint->GetScopedName(), this->pidWheels);
    }

        /*
         * Subscribe to topic
         * from soMap
         * @ key - topic ket from soMap
        */
    private: void subscribe(std::string key)
    {
            // Get subscribe option
            auto so = this->soMap[key];
            // Subscribe to it
            this->rosSub = this->rosNodeSub->subscribe(so);
    }
 };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(OmniPlatformPlugin)
}


