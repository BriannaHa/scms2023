#include "sample.h"
#include <cmath>

using namespace cv;


// Constructor assigns values to local variables when an object of this class is called
Sample::Sample(ros::NodeHandle nh) : goalSet_(false), laserProcessingPtr_(nullptr) {

    // subscribers
    subOdom_ = nh_.subscribe("/odom", 1000, &Sample::odoCallback,this); 
    subImages_ = nh_.subscribe("/camera/rgb/image_raw", 10, &Sample::imageCallback,this); 
    subDepth_ = nh_.subscribe("/camera/depth/points", 10, &Sample::depthCallback,this); 
    subCamINFO_ = nh_.subscribe("/camera/rgb/camera_info", 10, &Sample::depthCallback,this);

    //publishers
    pubCmdVel_  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",3,false);

}

// Destructor destroys any existing LaserProcessing objects
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
    // a mutex is used to protect the image_ variable when it is being written to (cannot be accessed and read elsewhere)
    std::unique_lock<std::mutex> lck(imageMtx_);

    // the internal image variable is set to the image data received by the subscriber
    image_ = *msg; 
    lck.unlock();
}

void Sample::depthCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // a mutex is used to protect the depth_ variable when it is being written to (cannot be accessed and read elsewhere)
    std::unique_lock<std::mutex> lck(depthMtx_);

    // the internal depth variable is set to the depth data received by the subscriber
    depth_ = *msg; 
    lck.unlock();
}

nav_msgs::Odometry Sample::getOdometry(void){
    // a mutex is used to protect the odometry variable when it is being read (cannot be accessed and written to elsewhere)
    std::unique_lock<std::mutex> lck1 (odoMtx_);
    return odo_;
}

void Sample::camINFOCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    // a mutex is used to protect the camera info variable when it is being written to (cannot be accessed and read elsewhere)
    std::unique_lock<std::mutex> lck(cameraINFOMtx_);

    // the internal camera info variable is set to the camera information received by the subscriber
    camInfo_ = *msg; 
    lck.unlock();
}

void Sample::test() {
    geometry_msgs::Twist base_cmd;
    findSquare(); // function to detect square
    reachSquare(); // function to travel to square's normal
    travelPerpendicular(); // function to travel straight, perpendicular to the square

    // velocities set to zero to stop the Turtlebot from moving after program has finished
    base_cmd.linear.x = 0;
    base_cmd.linear.y = 0;
    base_cmd.angular.z = 0;
    pubCmdVel_.publish(base_cmd);
}


// The following function commands the Turtlebot to keep rotating until a square is detected
void Sample::findSquare() {
    bool squareFound = false;
    
    // The following code runs while a square has not been detected
    while(!squareFound) {
        int count = 0;

        // 3 attempts are made to detect a square before the turtlebot moves to account for false negatives
        while(!squareFound && count<3) { 
            ros::Rate rate(5);
            geometry_msgs::Twist base_cmd;
            base_cmd.linear.x = 0; 
            base_cmd.linear.y = 0;
            base_cmd.angular.z = 0; 
            pubCmdVel_.publish(base_cmd);
            squareFound = detectSquare(); // function to detect square from current pose is called
            count++;

            // if a square isn't found, the robot moves a small amount to retest whether the previous location was a singularity that led to no detection
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

        // If a square isn't found, the Turtlebot rotates anticlockwise 45 degrees and attempts to detect a square from its new pose
        if(!squareFound) {
            ros::Rate rate(5);
            geometry_msgs::Twist base_cmd_turn_left;
            base_cmd_turn_left.linear.x = 0; 
            base_cmd_turn_left.linear.y = 0;
            base_cmd_turn_left.angular.z = M_PI_4/2;
            for(int n=10; n>0; n--) {
                pubCmdVel_.publish(base_cmd_turn_left);
                rate.sleep();
            }
        }
    }
}

// The following function creates a laserprocessing object to determine if a square is detected
bool Sample::detectSquare() {
    // mutexes are used to ensure that these variables are not being written to when they are passed into the LaserProcessing constructor
    std::unique_lock<std::mutex> lck1(imageMtx_);
    std::unique_lock<std::mutex> lck2(depthMtx_);
    std::unique_lock<std::mutex> lck3(cameraINFOMtx_);
    LaserProcessing laserProcessing(image_, depth_, camInfo_, odo_); // LaserProcessing object created
    lck1.unlock();
    lck2.unlock();
    lck3.unlock();

    bool test = laserProcessing.testMessages(y_int_, turnAngle_, distance_); // calls the testMessages function in the LaserProcessing class which returns true/false depending on whether a square has been detected
    cout << "Square detected: " << test << std::endl;

    return test;
}

// The following function programs the Turtlebot to move from its current location to the square's normal
void Sample::reachSquare() {
    double squareRot = -0.3;

    // Turn so that the Turtlebot is parallel to the square in the x-y frame
    std::cout << "Turning Parallel to Square" << std::endl;
    geometry_msgs::Twist base_cmd_turn_parallel;
    base_cmd_turn_parallel.linear.x = 0; 
    base_cmd_turn_parallel.linear.y = 0;
    base_cmd_turn_parallel.angular.z = 0.5;
    double rot = squareRot+M_PI;
    // Rotation speed decreases as the Turtlebot approaches the desired orientation to increase accuracy
    while(abs(rot-getYaw(odo_.pose.pose.orientation)) >= 0.5) {
        pubCmdVel_.publish(base_cmd_turn_parallel);
    }
    base_cmd_turn_parallel.angular.z = 0.2;
    while(abs(rot-getYaw(odo_.pose.pose.orientation)) >= 0.2) {
        pubCmdVel_.publish(base_cmd_turn_parallel);
    }
    base_cmd_turn_parallel.angular.z = 0.05;
    while(abs(rot-getYaw(odo_.pose.pose.orientation)) >= 0.05) {
        pubCmdVel_.publish(base_cmd_turn_parallel);
    }
    base_cmd_turn_parallel.angular.z = 0.025;
    while(abs(rot-getYaw(odo_.pose.pose.orientation)) >= 0.005) {
        pubCmdVel_.publish(base_cmd_turn_parallel);
    }

    // Travel forward until the Turtlebot reaches the square's normal
    std::cout << "Travel to Square Normal" << std::endl;
    // Calculate the equation of the normal line in the x-y plane
    double y = 7;
    double x = -1;
    double m = tan(squareRot);
    double mn = -1/m;
    double b = y-mn*x;
    geometry_msgs::Twist base_cmd_travel_parallel;
    base_cmd_travel_parallel.linear.x = 0.2; 
    base_cmd_travel_parallel.linear.y = 0;
    base_cmd_travel_parallel.angular.z = 0;
    // Linear speed decreases as the Turtlebot approaches the desired position to increase accuracy
    while(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b) >= 0.2) {
        pubCmdVel_.publish(base_cmd_travel_parallel);
    }
    base_cmd_travel_parallel.linear.x = 0.05; 
    while(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b) >= 0.05) {
        pubCmdVel_.publish(base_cmd_travel_parallel);
    }
    base_cmd_travel_parallel.linear.x = 0.005; 
    while(abs(mn*odo_.pose.pose.position.x-odo_.pose.pose.position.y+b) >= 0.005) {
        pubCmdVel_.publish(base_cmd_travel_parallel);
    }

    // Turn so that the Turtlebot aligns with the normal and faces the square
    std::cout << "Turning to Face Square" << std::endl;
    geometry_msgs::Twist base_cmd_turn_normal;
    base_cmd_turn_normal.linear.x = 0; 
    base_cmd_turn_normal.linear.y = 0;
    base_cmd_turn_normal.angular.z = -0.5;
    double rot2 = (M_PI/2)+squareRot;
    // Rotation speed decreases as the Turtlebot approaches the desired orientation to increase accuracy
    while(abs(rot2-getYaw(odo_.pose.pose.orientation)) >= 0.5) {
        pubCmdVel_.publish(base_cmd_turn_normal);
    }
    base_cmd_turn_normal.angular.z = -0.2;
    while(abs(rot2-getYaw(odo_.pose.pose.orientation)) >= 0.2) {
        pubCmdVel_.publish(base_cmd_turn_normal);
    }
    base_cmd_turn_normal.angular.z = -0.05;
    while(abs(rot2-getYaw(odo_.pose.pose.orientation)) >= 0.05) {
        pubCmdVel_.publish(base_cmd_turn_normal);
    }
    base_cmd_turn_normal.angular.z = -0.025;
    while(abs(rot2-getYaw(odo_.pose.pose.orientation)) >= 0.005) {
        pubCmdVel_.publish(base_cmd_turn_normal);
    }

    std::cout << "Ready to Track Square" << std::endl;
}


// The following function programs the Turtlebot to travel along the square's normal and adjust its trajectory if it starts going off-course
void Sample::travelPerpendicular() {
    // mutexes are used to ensure that these variables are not being written to when they are passed into the LaserProcessing constructor
    std::unique_lock<std::mutex> lck1(imageMtx_);
    std::unique_lock<std::mutex> lck2(depthMtx_);
    std::unique_lock<std::mutex> lck3(cameraINFOMtx_);
    LaserProcessing laserProcessing(image_, depth_, camInfo_, odo_); // LaserProcessing object created
    lck1.unlock();
    lck2.unlock();
    lck3.unlock();

    double depth = 0;
    double turnAngle = 0;
    laserProcessing.findCentre(depth, turnAngle); // calls the findCentre function in the LaserProcessing class which passes back via reference the distance (depth) to the square and the angle between the the camera's centre and the square's centre

    // The following sets the forward velocity to 0.25 and the turn angle to the relative angle between the square's centre and the camera's centre
    geometry_msgs::Twist base_cmd_travel_normal;
    base_cmd_travel_normal.linear.x = 0.25; 
    base_cmd_travel_normal.linear.y = 0;
    base_cmd_travel_normal.angular.z = turnAngle;
    
    // The Turtlebot is programmed to keep moving forwards until its distance is less than 1.5m away from the square
    while(depth > 1.5) {
        pubCmdVel_.publish(base_cmd_travel_normal); // velocity message published to command Turtlebot platform to move

        // mutexes are used to ensure that these variables are not being written to when they are passed into the LaserProcessing constructor
        std::unique_lock<std::mutex> lck4(imageMtx_);
        std::unique_lock<std::mutex> lck5(depthMtx_);
        std::unique_lock<std::mutex> lck6(cameraINFOMtx_);
        LaserProcessing laserProcessing2(image_, depth_, camInfo_, odo_); // LaserProcessing object created
        lck4.unlock();
        lck5.unlock();
        lck6.unlock();
        laserProcessing2.findCentre(depth, turnAngle); // calls the findCentre function in the LaserProcessing class which passes back via reference the distance (depth) to the square and the angle between the the camera's centre and the square's centre

        // The following sets the forward velocity to 0.25 and the turn angle to the relative angle between the square's centre and the camera's centre
        base_cmd_travel_normal.linear.x = 0.25; 
        base_cmd_travel_normal.linear.y = 0;
        base_cmd_travel_normal.angular.z = turnAngle;
    }
    
    // velocities set to zero to stop the Turtlebot from moving after program has finished
    base_cmd_travel_normal.linear.x = 0;
    base_cmd_travel_normal.linear.y = 0;
    base_cmd_travel_normal.angular.z = 0;
    pubCmdVel_.publish(base_cmd_travel_normal);
}


// The following function converts the quaternion returned from the odometry orientation and returns the calculated yaw value in radians
double Sample::getYaw(geometry_msgs::Quaternion q) {
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}