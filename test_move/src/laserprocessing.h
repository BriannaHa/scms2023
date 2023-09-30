#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>


#include <math.h>
#include "nav_msgs/Odometry.h"

class LaserProcessing {
    public:
        LaserProcessing(sensor_msgs::Image image, sensor_msgs::PointCloud2 depth);
        void testMessages();
    
    private:
        sensor_msgs::Image image_;//!< laser data passed in when an object of this class is initialised
        sensor_msgs::PointCloud2 depth_;
};


#endif // DETECTCABINET_H