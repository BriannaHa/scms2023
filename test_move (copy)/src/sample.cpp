#include "sample.h"
#include <cmath>

using namespace cv;
// using namespace cv::features2d;

Sample::Sample(ros::NodeHandle nh) : goalSet_(false), laserProcessingPtr_(nullptr) {

    // subscribers
    subOdom_ = nh_.subscribe("/odom", 1000, &Sample::odoCallback,this); 
    subImages_ = nh_.subscribe("/camera/rgb/image_raw", 10, &Sample::imageCallback,this); 
    subDepth_ = nh_.subscribe("/camera/depth/points", 10, &Sample::depthCallback,this); 

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

    base_cmd.linear.x = 0;
    base_cmd.linear.y = 0;
    base_cmd.angular.z = 0;
    base_cmd_turn_left.linear.x = 0; 
    base_cmd_turn_left.linear.y = 0;
    base_cmd_turn_left.angular.z = 0;


    //and let's go forward by setting X to a positive value
    base_cmd.linear.x = 0.25;
    base_cmd.angular.z = 0.0;

    //base_cmd_turn_left will be used to turn turtlebot 90 degrees
    base_cmd_turn_left.linear.x = 0; //m/s
    base_cmd_turn_left.angular.z = 1.57/2; //45 deg/s * 2 sec = 90 degrees 

    ros::Rate rate(5); // 5Hz
    for(int i=0; i<10; i++) { //have we ctrl + C?  If no... keep going!
        //"publish" sends the command to turtlebot to keep going

        std::unique_lock<std::mutex> lck1(imageMtx_);
        std::unique_lock<std::mutex> lck2(depthMtx_);
        // LaserProcessing laserProcessing(getImage(), getDepth());
        LaserProcessing laserProcessing(image_, depth_);
        lck1.unlock();
        lck2.unlock();

        laserProcessing.testMessages();

        //go forward for 2 seconds
        for(int n=10; n>0; n--) {
            pubCmdVel_.publish(base_cmd);
            rate.sleep();
        }

        //turn 90 degrees (takes 2 seconds)
        for(int n=10; n>0; n--) {
            pubCmdVel_.publish(base_cmd_turn_left);
            rate.sleep();
        }
    }
    base_cmd.linear.x = 0;
    base_cmd.linear.y = 0;
    base_cmd.angular.z = 0;
    pubCmdVel_.publish(base_cmd);

    // std::unique_lock<std::mutex> lck1(imageMtx_);
    // std::unique_lock<std::mutex> lck2(depthMtx_);
    // // LaserProcessing laserProcessing(getImage(), getDepth());
    // LaserProcessing laserProcessing(image_, depth_);
    // lck1.unlock();
    // lck2.unlock();

    // laserProcessing.testMessages();
}

void Sample::detectSquare() {

}

void Sample::reachSquare() {

}

void Sample::travelPerpendicular() {

}