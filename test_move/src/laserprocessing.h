#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <vector>

#include <math.h>
#include "nav_msgs/Odometry.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

using namespace cv;
using namespace std;

class LaserProcessing {
    public:
        LaserProcessing(sensor_msgs::Image image, sensor_msgs::PointCloud2 depth, sensor_msgs::CameraInfo info, nav_msgs::Odometry odo);
        bool testMessages(double& y_int, double& turnAngle, double& distance); // main function to detect square within for simulation test
        void convertToGreyscale(); // function to convert RGB image from camera into Greyscale for Harris Corner Detection
        void saveGreyscaleAsPGM(const std::string& filename, const std::vector<uint8_t>& grayscale_image, int width, int height); // function to save Greyscale image for viewing during testing
        void cornerHarris_demo( int, void*); // function to detect corners in image
        static void staticCornerHarris(int value, void* userdata); // function to detect corners in image
        void setSquareDetected(bool squareDetected); // function to toggle state of square detection
        void calculate3DCoords(); // function to calculate 3D coordinates (local and global) from 2D camera coordinates
        void normalAngleToBot(); // function to calculate normal vector and relative angle between square's normal and Turtlebot
        double getYaw(geometry_msgs::Quaternion q); // function to convert Turtlebot's pose quaternion to yaw for angles
        void findNormal(); //function to calculate find the equation of the square's normal
        void findCentre(double& depth, double& turnAngle); // function to find the centre of the square and turn angle to realign with camera centre

        struct CoordPoint { // struct to store 2D coordinates
            int x, y;
        };
        struct CoordPoint3D { //struct to store 3D coordinates
            double x, y, z;
        };

        CoordPoint3D localToGlobal(LaserProcessing::CoordPoint3D localPoint); //convert 3D local coordinates to 3D global coordinates
        CoordPoint3D rotateByYaw(LaserProcessing::CoordPoint3D localPoint, double yaw); //convert 3D local coordinates by applying yaw rotation
    
    private:
        sensor_msgs::Image image_; // Stores RGB image data received from RGB Camera/imageraw topic locally
        sensor_msgs::PointCloud2 depth_; // Stores depth data received from RGBD Camera/imageraw topic locally
        sensor_msgs::CameraInfo camInfo_; // Stores intrinsic and extrinsic parameters of the RGB camera
        nav_msgs::Odometry odo_; // odometry data for the Turtlebot's current position
        std::vector<uint8_t> greyscale_image_; // the grayscale image (after conversion)
        bool greyscale_image_initialised_; // boolean status of greyscale conversion

        Mat src_gray; 
        int thresh = 190;
        int max_thresh = 255;
        const char* source_window = "Source image";
        const char* corners_window = "Corners detected";
        std::vector<int> corner_x_coords; // Vector to save x-coordinates of corners
        std::vector<int> corner_y_coords; // Vector to save y-coordinates of corners
        bool squareDetected_; // boolean status of square detection

        std::vector<int> squareCornersX_; //square corners in camera's frame (pixel coordinates of X -left/right)
        std::vector<int> squareCornersY_; //square corners in camera's frame (pixel coordiantes of Y - top/bottom)

        double centreX_, centreY_; // square centre coordinates for tracking

        CoordPoint topLeft_, topRight_, bottomLeft_, bottomRight_; // square coordinates in 2D (camera frame)
        CoordPoint3D topLeft3D_, topRight3D_, bottomLeft3D_, bottomRight3D_; // square coordinates in 3D (camera frame)
        CoordPoint3D topLeft3DGlobal_, topRight3DGlobal_, bottomLeft3DGlobal_, bottomRight3DGlobal_, centreGlobal_; // square coordinates in 3D (global coordinates)

        double y_int_; // y-intercept of equation for square's normal

        double thetaRad_; // angle between plane normal and camera normal

        double angleGradient_; // angle to the square's normal
        double angleParallel_; // angle to be parallel to the square's plane
        double distance_; // distance to reach the square's normal
};


#endif