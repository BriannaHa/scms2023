#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
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
        LaserProcessing(sensor_msgs::Image image, sensor_msgs::PointCloud2 depth);
        bool testMessages();
        void convertToGreyscale();
        void saveGreyscaleAsPGM(const std::string& filename, const std::vector<uint8_t>& grayscale_image, int width, int height);
        void cornerHarris_demo( int, void*);
        static void staticCornerHarris(int value, void* userdata);
        void setSquareDetected(bool squareDetected);
    
    private:
        sensor_msgs::Image image_;//!< laser data passed in when an object of this class is initialised
        sensor_msgs::PointCloud2 depth_;
        std::vector<uint8_t> greyscale_image_; //!< the grayscale image (after conversion)
        bool greyscale_image_initialised_; //!<

        Mat src_gray;
        int thresh = 150;
        int max_thresh = 255;
        const char* source_window = "Source image";
        const char* corners_window = "Corners detected";
        std::vector<int> corner_x_coords; // Vector to save x-coordinates of corners
        std::vector<int> corner_y_coords; // Vector to save y-coordinates of corners
        bool squareDetected_;

};


#endif