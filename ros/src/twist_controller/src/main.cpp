#include "LowLevelControllerNode.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "low_level_controller_node");

  // Create node handle to read parameters
  auto param_node_handle = ros::NodeHandle("~");

  // Read rate
  int rate_freq = 50;
  param_node_handle.param<int>("rate", rate_freq, rate_freq);
  ros::Rate rate(rate_freq);

  // Main loop
  LowLevelControllerNode low_level_controller_node;
  while (ros::ok()) {
    low_level_controller_node.process();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
