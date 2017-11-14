#ifndef _LOW_LEVEL_CONTROLLER_NODE_H_
#define _LOW_LEVEL_CONTROLLER_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/SteeringReport.h>

#include <std_msgs/Bool.h>
#include <styx_msgs/Lane.h>

class LowLevelControllerNode {
  // Attributes
  dbw_mkz_msgs::ThrottleCmd m_throttle_cmd;
  dbw_mkz_msgs::SteeringCmd m_steering_cmd;
  dbw_mkz_msgs::BrakeCmd m_brake_cmd;

  styx_msgs::Lane m_final_waypoints;
  geometry_msgs::TwistStamped m_current_velocity;
  geometry_msgs::PoseStamped m_current_pose;
  bool m_dbw_enabled;

  double m_vehicle_mass;
  double m_fuel_capacity;
  double m_brake_deadband;
  double m_decel_limit;
  double m_accel_limit;
  double m_wheel_radius;
  double m_wheel_base;
  double m_steer_ratio;
  double m_max_lat_accel;
  double m_max_steer_angle;


  // Node handle
  ros::NodeHandle m_node_handle;

  // Published topics
  ros::Publisher m_steer_pub;
  ros::Publisher m_throttle_pub;
  ros::Publisher m_brake_pub;

  // Subsribed topics
  ros::Subscriber m_sub_final_waypoints;
  ros::Subscriber m_sub_current_velocity;
  ros::Subscriber m_sub_current_pose;
  ros::Subscriber m_sub_dbw_enabled;


  // Methods
  void init();
  void receiveFinalWaypointsCallback(const styx_msgs::Lane &data);
  void receiveCurrentVelocityCallback(const geometry_msgs::TwistStamped &data);
  void receiveCurrentPoseCallback(const geometry_msgs::PoseStamped &data);
  void receiveDbwEnabledCallback(const std_msgs::Bool &msg);

public:
  LowLevelControllerNode() { init(); }

  void process();
};

#endif // _LOW_LEVEL_CONTROLLER_NODE_H_
