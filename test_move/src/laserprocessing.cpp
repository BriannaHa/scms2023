#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::Image image, sensor_msgs::PointCloud2 depth):
    image_(image), depth_(depth)
{
}

void LaserProcessing::testMessages() {
    ROS_INFO_STREAM("image test");
    ROS_INFO_STREAM(image_.header.seq);
    ROS_INFO_STREAM("depth test");
    ROS_INFO_STREAM(depth_.header.seq);
}

