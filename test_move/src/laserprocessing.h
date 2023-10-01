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

class LaserProcessing {
    public:
        LaserProcessing(sensor_msgs::Image image, sensor_msgs::PointCloud2 depth);
        void testMessages();
        void convertToGreyscale();
        void saveGreyscaleAsPGM(const std::string& filename, const std::vector<uint8_t>& grayscale_image, int width, int height);
    
    private:
        sensor_msgs::Image image_;//!< laser data passed in when an object of this class is initialised
        sensor_msgs::PointCloud2 depth_;
        std::vector<uint8_t> greyscale_image_; //!< the grayscale image (after conversion)
        bool greyscale_image_initialised_; //!<
};


#endif