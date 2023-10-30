#include <ros/ros.h>
#include <thread>

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "sample.h"


int main(int argc, char **argv)
{
  // initialises a ROS node called test_move
  ros::init(argc, argv, "test_move");

  // creates a new node handle called nh
  ros::NodeHandle nh;

  // creates a shared pointed to an object of class Sample
  std::shared_ptr<Sample> testPtr(new Sample(nh));

  // a seperate thread is started for the test function to run (non-blocking function)
  std::thread t(&Sample::test, testPtr);

  // blocks the main thread from exiting until "Ctrl + C" is pressed (ROS shuts down)
  ros::spin();

  // shuts down ROS
  ros::shutdown();

  // thread is joined to ensure it finishes running before the main ends
  t.join();

  return 0;
}