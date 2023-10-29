// #ifndef SAMPLE_H
// #define SAMPLE_H

// #include <ros/ros.h>
// #include <atomic>
// #include <mutex>

// #include "laserprocessing.h"

// #include <iostream>
// #include <stdio.h>
// #include <cmath>

// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
// #include "geometry_msgs/Point.h"
// #include "geometry_msgs/PoseArray.h"
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/PointCloud2.h>

// #include "opencv2/core.hpp"
// #include "opencv2/features2d.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"
// #include <cv_bridge/cv_bridge.h>
// // #include "opencv2/xfeatures2d.hpp"



// class Sample{
//     public:
//         Sample(ros::NodeHandle nh);
//         ~Sample();
//         // void reachGoal(void);
//         // void setGoal(const geometry_msgs::PoseArray::ConstPtr& msg);
//         nav_msgs::Odometry getOdometry(void);
//         // sensor_msgs::Image getImage(void);
//         // sensor_msgs::PointCloud2 getDepth(void);
//         void odoCallback(const nav_msgs::Odometry::ConstPtr& msg);
//         void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
//         void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
//         bool detectSquare();
//         void reachSquare();
//         void travelPerpendicular();
//         void test();
//         void findSquare();
    
//     private:
//         nav_msgs::Odometry odo_;
//         geometry_msgs::Point goal_;
//         ros::NodeHandle nh_; 
//         ros::Subscriber subOdom_; //!< Subscriber to odometry messages
//         ros::Subscriber subGoals_; //!< Subscriber to goals set for the platform
//         ros::Subscriber subImages_; //!< Subscriber that subscribes to laser topic
//         ros::Subscriber subDepth_; //!< Subscriber that subscribes to laser topic
//         std::mutex odoMtx_;

//         LaserProcessing* laserProcessingPtr_;
//         double target_angle_ = 0; //!< Angle from quadcopter's current orientation to the goal
//         ros::Publisher pubCmdVel_; //!< Publisher to publish to a topic that commands the quadcopter's velocity
//         sensor_msgs::Image image_;//!< Stores laser data received from laser topic locally
//         sensor_msgs::Image depth_;//!< Stores laser data received from laser topic locally
//         std::mutex imageMtx_; //!< Mutex to lock Laser Data
//         std::mutex depthMtx_;

//         bool goalSet_;
//         double angle_;

//         int turn_;
//         bool left_;
//         bool right_;
//         bool perpendicular_;

// };

// #endif // SAMPLE_H

#ifndef SAMPLE_H
#define SAMPLE_H

#include <ros/ros.h>
#include <atomic>
#include <mutex>

#include "laserprocessing.h"

#include <iostream>
#include <stdio.h>
#include <cmath>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/CameraInfo.h"

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
// #include "opencv2/xfeatures2d.hpp"



class Sample{
    public:
        Sample(ros::NodeHandle nh);
        ~Sample();
        // void reachGoal(void);
        // void setGoal(const geometry_msgs::PoseArray::ConstPtr& msg);
        nav_msgs::Odometry getOdometry(void);
        // sensor_msgs::Image getImage(void);
        // sensor_msgs::PointCloud2 getDepth(void);
        void odoCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
        void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void camINFOCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
        bool detectSquare();
        void reachSquare();
        void travelPerpendicular();
        void test();
        void findSquare();
        double getYaw(geometry_msgs::Quaternion q);
    
    private:
        nav_msgs::Odometry odo_;
        geometry_msgs::Point goal_;
        ros::NodeHandle nh_; 
        ros::Subscriber subOdom_; //!< Subscriber to odometry messages
        ros::Subscriber subGoals_; //!< Subscriber to goals set for the platform
        ros::Subscriber subImages_; //!< Subscriber that subscribes to laser topic
        ros::Subscriber subDepth_; //!< Subscriber that subscribes to laser topic
        ros::Subscriber subCamINFO_; //!< Subscriber toc camera intrinsic and projection matrices
        std::mutex odoMtx_;

        LaserProcessing* laserProcessingPtr_;
        double target_angle_ = 0; //!< Angle from quadcopter's current orientation to the goal
        ros::Publisher pubCmdVel_; //!< Publisher to publish to a topic that commands the quadcopter's velocity
        sensor_msgs::Image image_;//!< Stores laser data received from laser topic locally
        sensor_msgs::PointCloud2 depth_;//!< Stores laser data received from laser topic locally
        sensor_msgs::CameraInfo camInfo_;
        std::mutex imageMtx_; //!< Mutex to lock Laser Data
        std::mutex depthMtx_;
        std::mutex cameraINFOMtx_;

        bool goalSet_;

        double y_int_, turnAngle_;

        double distance_;
};

#endif // SAMPLE_H