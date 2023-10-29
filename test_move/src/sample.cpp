#include "sample.h"
#include <cmath>

using namespace cv;
// using namespace cv::features2d;

Sample::Sample(ros::NodeHandle nh) : goalSet_(false), laserProcessingPtr_(nullptr) {

    // subscribers
    subOdom_ = nh_.subscribe("/odom", 1000, &Sample::odoCallback,this); 
    subImages_ = nh_.subscribe("/camera/rgb/image_raw", 10, &Sample::imageCallback,this); 
    subDepth_ = nh_.subscribe("/camera/depth/points", 10, &Sample::depthCallback,this); 
    subCamINFO_ = nh_.subscribe("/camera/rgb/camera_info", 10, &Sample::depthCallback,this);

    //publishers
    pubCmdVel_  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",3,false);

}

Sample::~Sample()
{
    if(laserProcessingPtr_ != nullptr){
        delete laserProcessingPtr_;
    }
}

void Sample::odoCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // a mutex is used to protect the odometry variable when it is being written to (cannot be accessed and read elsewhere)
    std::unique_lock<std::mutex> lck1 (odoMtx_);

    // the internal odometry variable is set to the odometry message received by the subscriber
    odo_ = *msg;
}

void Sample::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // a mutex is used to protect the laserData_ variable when it is being written to (cannot be accessed and read elsewhere)
    std::unique_lock<std::mutex> lck(imageMtx_);
    image_ = *msg; 
    lck.unlock();
}

void Sample::depthCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // a mutex is used to protect the laserData_ variable when it is being written to (cannot be accessed and read elsewhere)
    std::unique_lock<std::mutex> lck(depthMtx_);
    depth_ = *msg; 
    lck.unlock();
}

nav_msgs::Odometry Sample::getOdometry(void){
    // a mutex is used to protect the odometry variable when it is being read (cannot be accessed and written to elsewhere)
    std::unique_lock<std::mutex> lck1 (odoMtx_);
    return odo_;
}

void Sample::camINFOCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    // a mutex is used to protect the laserData_ variable when it is being written to (cannot be accessed and read elsewhere)
    std::unique_lock<std::mutex> lck(cameraINFOMtx_);
    camInfo_ = *msg; 
    lck.unlock();
}

// sensor_msgs::PointCloud2 Sample::getDepth(void){
//     // a mutex is used to protect the odometry variable when it is being read (cannot be accessed and written to elsewhere)
//     std::unique_lock<std::mutex> lck1 (depthMtx_);
//     return depth_;
// }

// sensor_msgs::Image Sample::getImage(void){
//     // a mutex is used to protect the odometry variable when it is being read (cannot be accessed and written to elsewhere)
//     std::unique_lock<std::mutex> lck1 (imageMtx_);
//     return image_;
// }

void Sample::test() {
    //init direction that turtlebot should go
    geometry_msgs::Twist base_cmd;
    geometry_msgs::Twist base_cmd_turn_left;
    findSquare();
    reachSquare();
    travelPerpendicular();
    // base_cmd.linear.x = 0;
    // base_cmd.linear.y = 0;
    // base_cmd.angular.z = 0;
    // base_cmd_turn_left.linear.x = 0; 
    // base_cmd_turn_left.linear.y = 0;
    // base_cmd_turn_left.angular.z = 0;


    // //and let's go forward by setting X to a positive value
    // base_cmd.linear.x = 0.25;
    // base_cmd.angular.z = 0.0;

    // //base_cmd_turn_left will be used to turn turtlebot 90 degrees
    // base_cmd_turn_left.linear.x = 0; //m/s
    // base_cmd_turn_left.angular.z = 1.57/2; //45 deg/s * 2 sec = 90 degrees 

    // ros::Rate rate(5); // 5Hz
    // for(int i=0; i<2; i++) { //have we ctrl + C?  If no... keep going!
    //     //"publish" sends the command to turtlebot to keep going

    //     std::unique_lock<std::mutex> lck1(imageMtx_);
    //     std::unique_lock<std::mutex> lck2(depthMtx_);
    //     // LaserProcessing laserProcessing(getImage(), getDepth());
    //     LaserProcessing laserProcessing(image_, depth_);
    //     lck1.unlock();
    //     lck2.unlock();

    //     bool test = laserProcessing.testMessages();
    //     cout << "Square detected: " << test << std::endl;
    //     base_cmd.linear.x = 0;
    // base_cmd.linear.y = 0;
    // base_cmd.angular.z = 0;
    // pubCmdVel_.publish(base_cmd);

    //     //go forward for 2 seconds
    //     for(int n=10; n>0; n--) {
    //         pubCmdVel_.publish(base_cmd);
    //         rate.sleep();
    //     }

    //     //turn 90 degrees (takes 2 seconds)
    //     for(int n=10; n>0; n--) {
    //         pubCmdVel_.publish(base_cmd_turn_left);
    //         rate.sleep();
    //     }
    //     base_cmd.linear.x = 0;
    // base_cmd.linear.y = 0;
    // base_cmd.angular.z = 0;
    // pubCmdVel_.publish(base_cmd);
    // }
    base_cmd.linear.x = 0;
    base_cmd.linear.y = 0;
    base_cmd.angular.z = 0;
    pubCmdVel_.publish(base_cmd);
}

void Sample::findSquare() {
    bool squareFound = false;
    
    while(!squareFound) {
        int count = 0;
        while(!squareFound && count<3) {
            ros::Rate rate(5);
            geometry_msgs::Twist base_cmd;
            base_cmd.linear.x = 0; 
            base_cmd.linear.y = 0;
            base_cmd.angular.z = 0; 
            pubCmdVel_.publish(base_cmd);
            squareFound = detectSquare();
            count++;
            if(!squareFound) {
                base_cmd.linear.x = 0.01; 
                base_cmd.linear.y = 0;
                base_cmd.angular.z = 0; 
                for(int n=10; n>0; n--) {
                    pubCmdVel_.publish(base_cmd);
                    rate.sleep();
                }
            }
        }

        if(!squareFound) {
            ros::Rate rate(5);
            geometry_msgs::Twist base_cmd_turn_left;
            base_cmd_turn_left.linear.x = 0; 
            base_cmd_turn_left.linear.y = 0;
            base_cmd_turn_left.angular.z = M_PI_4/2; //45 deg/s * 2 sec = 90 degrees 
            for(int n=10; n>0; n--) {
                pubCmdVel_.publish(base_cmd_turn_left);
                rate.sleep();
            }
        }
    }
    
}

bool Sample::detectSquare() {
    std::unique_lock<std::mutex> lck1(imageMtx_);
    std::unique_lock<std::mutex> lck2(depthMtx_);
    std::unique_lock<std::mutex> lck3(cameraINFOMtx_);
        // LaserProcessing laserProcessing(getImage(), getDepth());
    LaserProcessing laserProcessing(image_, depth_, camInfo_, odo_);
    lck1.unlock();
    lck2.unlock();
    lck3.unlock();

    bool test = laserProcessing.testMessages(y_int_, turnAngle_, distance_);
    cout << "Square detected: " << test << std::endl;
    if(test) {
        cout << y_int_ << endl;
        cout << turnAngle_ << endl;
        cout << distance_ << endl;
    }
    return test;
}

void Sample::reachSquare() {
    // geometry_msgs::Twist base_cmd_turn_parallel;
    // base_cmd_turn_parallel.linear.x = 0; 
    // base_cmd_turn_parallel.linear.y = 0;
    // base_cmd_turn_parallel.angular.z = -0.1;
    // ros::Rate rate(5);
    // double rot = -0.3;
    // while(abs(rot-getYaw(odo_.pose.pose.orientation)) >= 0.05) {
    //     pubCmdVel_.publish(base_cmd_turn_parallel);
    //     // rate.sleep();
    // }
    double squareRot = -0.3;

    // Turn Parallel
    geometry_msgs::Twist base_cmd_turn_parallel;
    base_cmd_turn_parallel.linear.x = 0; 
    base_cmd_turn_parallel.linear.y = 0;
    base_cmd_turn_parallel.angular.z = 0.5;
    
    double rot = squareRot+M_PI;
    while(abs(rot-getYaw(odo_.pose.pose.orientation)) >= 0.5) {
        pubCmdVel_.publish(base_cmd_turn_parallel);
        ROS_INFO_STREAM(abs(rot-getYaw(odo_.pose.pose.orientation)));
        // rate.sleep();
    }
    base_cmd_turn_parallel.angular.z = 0.2;
    while(abs(rot-getYaw(odo_.pose.pose.orientation)) >= 0.2) {
        pubCmdVel_.publish(base_cmd_turn_parallel);
        ROS_INFO_STREAM(abs(rot-getYaw(odo_.pose.pose.orientation)));
        // rate.sleep();
    }
    base_cmd_turn_parallel.angular.z = 0.05;
    while(abs(rot-getYaw(odo_.pose.pose.orientation)) >= 0.05) {
        pubCmdVel_.publish(base_cmd_turn_parallel);
        ROS_INFO_STREAM(abs(rot-getYaw(odo_.pose.pose.orientation)));
        // rate.sleep();
    }
    base_cmd_turn_parallel.angular.z = 0.01;
    while(abs(rot-getYaw(odo_.pose.pose.orientation)) >= 0.005) {
        pubCmdVel_.publish(base_cmd_turn_parallel);
        ROS_INFO_STREAM(abs(rot-getYaw(odo_.pose.pose.orientation)));
        // rate.sleep();
    }

    double y = 7;
    double x = -1;

    double m = tan(squareRot);
    double mn = -1/m;
    double b = y-mn*x;
    ROS_INFO_STREAM(b);

    geometry_msgs::Twist base_cmd_travel_parallel;
    base_cmd_travel_parallel.linear.x = 0.5; 
    base_cmd_travel_parallel.linear.y = 0;
    base_cmd_travel_parallel.angular.z = 0;
    // while(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b) >= 0.5) {
    //     pubCmdVel_.publish(base_cmd_travel_parallel);
    //     // rate.sleep();
    //     ROS_INFO_STREAM(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b));
    // }
    base_cmd_travel_parallel.linear.x = 0.2; 
    while(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b) >= 0.2) {
        pubCmdVel_.publish(base_cmd_travel_parallel);
        // rate.sleep();
        ROS_INFO_STREAM(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b));
    }
    base_cmd_travel_parallel.linear.x = 0.05; 
    while(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b) >= 0.05) {
        pubCmdVel_.publish(base_cmd_travel_parallel);
        // rate.sleep();
        ROS_INFO_STREAM(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b));
    }
    base_cmd_travel_parallel.linear.x = 0.005; 
    while(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b) >= 0.005) {
        pubCmdVel_.publish(base_cmd_travel_parallel);
        // rate.sleep();
        ROS_INFO_STREAM(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b));
    }

    geometry_msgs::Twist base_cmd_turn_normal;
    base_cmd_turn_normal.linear.x = 0; 
    base_cmd_turn_normal.linear.y = 0;
    base_cmd_turn_normal.angular.z = -0.5;
    double rot2 = (M_PI/2)+squareRot;
    // if(rot2>M_PI) rot2 -= 2*M_PI;
    // while(abs(rot2-getYaw(odo_.pose.pose.orientation)) >= 0.05) {
    //     pubCmdVel_.publish(base_cmd_turn_normal);
    //     // rate.sleep();
    // }
    while(abs(rot2-getYaw(odo_.pose.pose.orientation)) >= 0.5) {
        pubCmdVel_.publish(base_cmd_turn_normal);
        ROS_INFO_STREAM(abs(rot2-getYaw(odo_.pose.pose.orientation)));
        // rate.sleep();
    }
    base_cmd_turn_normal.angular.z = -0.2;
    while(abs(rot2-getYaw(odo_.pose.pose.orientation)) >= 0.2) {
        pubCmdVel_.publish(base_cmd_turn_normal);
        ROS_INFO_STREAM(abs(rot2-getYaw(odo_.pose.pose.orientation)));
        // rate.sleep();
    }
    base_cmd_turn_normal.angular.z = -0.05;
    while(abs(rot2-getYaw(odo_.pose.pose.orientation)) >= 0.05) {
        pubCmdVel_.publish(base_cmd_turn_normal);
        ROS_INFO_STREAM(abs(rot2-getYaw(odo_.pose.pose.orientation)));
        // rate.sleep();
    }
    base_cmd_turn_normal.angular.z = -0.01;
    while(abs(rot2-getYaw(odo_.pose.pose.orientation)) >= 0.005) {
        pubCmdVel_.publish(base_cmd_turn_normal);
        ROS_INFO_STREAM(abs(rot2-getYaw(odo_.pose.pose.orientation)));
        // rate.sleep();
    }

    // ros::Rate rate(5);
    // geometry_msgs::Twist base_cmd_travel_normal;
    // base_cmd_travel_normal.linear.x = 2; 
    // base_cmd_travel_normal.linear.y = 0;
    // base_cmd_travel_normal.angular.z = 0;
    // double currentX = odo_.pose.pose.position.x;
    // double currentY = odo_.pose.pose.position.y;
    // while(abs(currentX-odo_.pose.pose.position.x) < 5 && abs(currentY-odo_.pose.pose.position.y) < 5) {
    //     pubCmdVel_.publish(base_cmd_travel_normal);
    //     ROS_INFO_STREAM("test");
    // }
}

void Sample::travelPerpendicular() {
    std::unique_lock<std::mutex> lck1(imageMtx_);
    std::unique_lock<std::mutex> lck2(depthMtx_);
    std::unique_lock<std::mutex> lck3(cameraINFOMtx_);
    LaserProcessing laserProcessing(image_, depth_, camInfo_, odo_);
    lck1.unlock();
    lck2.unlock();
    lck3.unlock();
    double depth = 0;
    double turnAngle = 0;
    laserProcessing.findCentre(depth, turnAngle);

    geometry_msgs::Twist base_cmd_travel_normal;
    base_cmd_travel_normal.linear.x = 0.25; 
    base_cmd_travel_normal.linear.y = 0;
    base_cmd_travel_normal.angular.z = turnAngle;
    // double currentX = odo_.pose.pose.position.x;
    // double currentY = odo_.pose.pose.position.y;
    while(depth > 1.5) {
        pubCmdVel_.publish(base_cmd_travel_normal);
        ROS_INFO_STREAM(depth);
        ROS_INFO_STREAM(turnAngle);

        std::unique_lock<std::mutex> lck4(imageMtx_);
        std::unique_lock<std::mutex> lck5(depthMtx_);
        std::unique_lock<std::mutex> lck6(cameraINFOMtx_);
        LaserProcessing laserProcessing2(image_, depth_, camInfo_, odo_);
        lck4.unlock();
        lck5.unlock();
        lck6.unlock();
        laserProcessing2.findCentre(depth, turnAngle);

        base_cmd_travel_normal.linear.x = 0.25; 
        base_cmd_travel_normal.linear.y = 0;
        base_cmd_travel_normal.angular.z = turnAngle;
    }
    base_cmd_travel_normal.linear.x = 0;
    base_cmd_travel_normal.linear.y = 0;
    base_cmd_travel_normal.angular.z = 0;
    pubCmdVel_.publish(base_cmd_travel_normal);
}

double Sample::getYaw(geometry_msgs::Quaternion q) {
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    // std::cout << "yaw: " << yaw << "rad" << std::endl;

    return yaw;
}