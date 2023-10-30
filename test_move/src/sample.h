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


class Sample{
    public:
        Sample(ros::NodeHandle nh);
        ~Sample();
        nav_msgs::Odometry getOdometry(void);
        void odoCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
        void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void camINFOCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
        bool detectSquare(); //function to detect whether there is a square in the camera's captured image
        void reachSquare(); //function to move Turtlebot to the square's projected normal 
        void travelPerpendicular(); //function to move Turtlebot along the line perpendicular to the square
        void test(); // main function to run the simulation
        void findSquare(); //function to keep turning the Turtlebot until a square is found
        double getYaw(geometry_msgs::Quaternion q); //function to convert Turtlebot pose quaternion to yaw for rotation
    
    private:
        nav_msgs::Odometry odo_; // odometry data for the Turtlebot's current position
        ros::NodeHandle nh_; // provides an interface for a software node to interact with the ROS middleware
        ros::Subscriber subOdom_; //!< Subscriber to odometry messages
        ros::Subscriber subGoals_; //!< Subscriber to goals set for the platform
        ros::Subscriber subImages_; //!< Subscriber that subscribes to laser topic
        ros::Subscriber subDepth_; //!< Subscriber that subscribes to laser topic
        ros::Subscriber subCamINFO_; //!< Subscriber toc camera intrinsic and projection matrices
        std::mutex odoMtx_;

        LaserProcessing* laserProcessingPtr_;
        ros::Publisher pubCmdVel_; // Publisher to publish to a topic that commands the Turtlebot's velocity
        sensor_msgs::Image image_; // Stores RGB image data received from RGB Camera/imageraw topic locally
        sensor_msgs::PointCloud2 depth_; // Stores depth data received from RGBD Camera/imageraw topic locally
        sensor_msgs::CameraInfo camInfo_; // Stores intrinsic and extrinsic parameters of the RGB camera
        std::mutex imageMtx_; // Mutex to lock Laser Data
        std::mutex depthMtx_; // Mutex to lock depth data
        std::mutex cameraINFOMtx_; // Mutex to lock camera info data

        bool goalSet_; // Boolean for whether goal is set for Turtlebot
        double y_int_, turnAngle_; // y-intercept and relative angle for square's normal vector
        double distance_; // distance to travel by Turtlebot
};

#endif // SAMPLE_H