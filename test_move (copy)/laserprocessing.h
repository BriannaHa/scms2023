// #ifndef LASERPROCESSING_H
// #define LASERPROCESSING_H

// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <iostream>
// #include <ros/ros.h>
// #include <stdio.h>
// #include <vector>

// #include <math.h>
// #include "nav_msgs/Odometry.h"

// #include "opencv2/highgui.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/imgcodecs.hpp"

// using namespace cv;
// using namespace std;

// class LaserProcessing {
//     public:
//         LaserProcessing(sensor_msgs::Image image, sensor_msgs::Image depth);
//         ~LaserProcessing();
//         bool testMessages(double& rotationAngle, int& turn, bool& left, bool& right);
//         void convertToGreyscale();
//         void saveGreyscaleAsPGM(const std::string& filename, const std::vector<uint8_t>& grayscale_image, int width, int height);
//         void cornerHarris_demo( int, void*);
//         static void staticCornerHarris(int value, void* userdata);
//         void setSquareDetected(bool squareDetected);
//         void findOrientation();

//         struct CoordPoint {
//             int x, y;
//         };

//         // Calculate Euclidean distance between two points
//         double distance( CoordPoint p1,  CoordPoint p2);

//         double findRotationAngle( CoordPoint topLeft,  CoordPoint topRight,
//                                  CoordPoint bottomLeft,  CoordPoint bottomRight);
    
//     private:
//         sensor_msgs::Image image_;//!< laser data passed in when an object of this class is initialised
//         sensor_msgs::Image depth_;
//         std::vector<uint8_t> greyscale_image_; //!< the grayscale image (after conversion)
//         bool greyscale_image_initialised_; //!<
//         std::vector<uint8_t> depth_image_;
//         std::vector<float> valid_depths_;

//         Mat src_gray;
//         int thresh = 190;
//         int max_thresh = 255;
//         const char* source_window = "Source image";
//         const char* corners_window = "Corners detected";
//         std::vector<int> corner_x_coords; // Vector to save x-coordinates of corners
//         std::vector<int> corner_y_coords; // Vector to save y-coordinates of corners
//         bool squareDetected_;
//         int x1_, x2_, y1_, y2_;
//         double rotationAngle_;

//         std::vector<int> squareCornersX_;
//         std::vector<int> squareCornersY_;

//         int turn_;
//         bool left_;
//         bool right_;
//         double centreX_, centreY_;

        

//         CoordPoint topLeft_, topRight_, bottomLeft_, bottomRight_;
// };


// #endif

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
        bool testMessages(double& y_int, double& turnAngle, double& distance);
        void convertToGreyscale();
        void saveGreyscaleAsPGM(const std::string& filename, const std::vector<uint8_t>& grayscale_image, int width, int height);
        void cornerHarris_demo( int, void*);
        static void staticCornerHarris(int value, void* userdata);
        void setSquareDetected(bool squareDetected);
        void calculate3DCoords();
        // void calculate3DNormal();
        // CoordPoint3D crossProduct(const CoordPoint3D& p1, const CoordPoint3D& p2);
        void normalAngleToBot();
        // void movementToPlaneNormal();
        double getYaw(geometry_msgs::Quaternion q);
        void findNormal();
        void findCentre(double& depth, double& turnAngle);
        
        


        struct CoordPoint {
            int x, y;
        };
        struct CoordPoint3D {
            double x, y, z;
        };
        CoordPoint3D local2Global(CoordPoint3D corner, nav_msgs::Odometry odo);
        CoordPoint3D localToGlobal(LaserProcessing::CoordPoint3D localPoint);
        CoordPoint3D rotateByYaw(LaserProcessing::CoordPoint3D localPoint, double yaw);
    
    private:
        sensor_msgs::Image image_;//!< laser data passed in when an object of this class is initialised
        sensor_msgs::PointCloud2 depth_;
        sensor_msgs::CameraInfo camInfo_;
        nav_msgs::Odometry odo_;
        std::vector<uint8_t> greyscale_image_; //!< the grayscale image (after conversion)
        bool greyscale_image_initialised_; //!<

        Mat src_gray;
        int thresh = 190;
        int max_thresh = 255;
        const char* source_window = "Source image";
        const char* corners_window = "Corners detected";
        std::vector<int> corner_x_coords; // Vector to save x-coordinates of corners
        std::vector<int> corner_y_coords; // Vector to save y-coordinates of corners
        bool squareDetected_;
        int x1_, x2_, y1_, y2_;
        double rotationAngle_;

        std::vector<int> squareCornersX_;
        std::vector<int> squareCornersY_;

        int turn_;
        bool left_;
        bool right_;
        double centreX_, centreY_;

        CoordPoint topLeft_, topRight_, bottomLeft_, bottomRight_;
        CoordPoint3D topLeft3D_, topRight3D_, bottomLeft3D_, bottomRight3D_;
        CoordPoint3D topLeft3DGlobal_, topRight3DGlobal_, bottomLeft3DGlobal_, bottomRight3DGlobal_, centreGlobal_;

        double y_int_;

        double thetaRad_; //angle between plane normal and camera normal
        double turnAngle_; 
        double travelDist_; 

        double angleGradient_, angleParallel_, distance_;
};


#endif