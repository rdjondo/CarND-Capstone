#include "LowLevelControllerNode.h"



#include <geometry_msgs/Pose2D.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <cmath>


using ::std::string;
using ::ros::NodeHandle;


using namespace std;


// Define configuration
struct LowLevelControllerNodeConf {

  double vehicle_mass = 1736.35;
  double fuel_capacity = 13.5;
  double brake_deadband = 0.1;
  double decel_limit = -5;
  double accel_limit = 1;
  double wheel_radius = 0.2413;
  double wheel_base = 2.8498;
  double steer_ratio = 14.8;
  double max_lat_accel =  3;
  double max_steer_angle = 8;

};

// Initialize node
void LowLevelControllerNode::init() {
  // Create default configuration
  LowLevelControllerNodeConf conf;

  // Create node handle to read parameters
  auto param_node_handle = NodeHandle("~");

  // Read parameters
  param_node_handle.param<double>("vehicle_mass", m_vehicle_mass, conf.vehicle_mass);
  param_node_handle.param<double>("fuel_capacity", m_fuel_capacity, conf.fuel_capacity);
  param_node_handle.param<double>("brake_deadband", m_brake_deadband, conf.brake_deadband);
  param_node_handle.param<double>("decel_limit", m_decel_limit, conf.decel_limit);
  param_node_handle.param<double>("accel_limit", m_accel_limit, conf.accel_limit);
  param_node_handle.param<double>("wheel_radius", m_wheel_radius, conf.wheel_radius);
  param_node_handle.param<double>("wheel_base", m_wheel_base, conf.wheel_base);
  param_node_handle.param<double>("steer_ratio", m_steer_ratio, conf.steer_ratio);
  param_node_handle.param<double>("max_lat_accel", m_max_lat_accel, conf.max_lat_accel);
  param_node_handle.param<double>("max_steer_angle", m_max_steer_angle, conf.max_steer_angle);


  // Set up publishers
  m_steer_pub =
      m_node_handle.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 1);

  m_throttle_pub =
      m_node_handle.advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 1);

  m_brake_pub =
      m_node_handle.advertise<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 1);


 
  // Set up subscribers
  m_sub_final_waypoints = m_node_handle.subscribe( "/final_waypoints", 1, 
     &LowLevelControllerNode::receiveFinalWaypointsCallback, this, ros::TransportHints().tcpNoDelay(true));

  m_sub_current_velocity = m_node_handle.subscribe( "/current_velocity", 1, 
     &LowLevelControllerNode::receiveCurrentVelocityCallback, this, ros::TransportHints().tcpNoDelay(true));

  m_sub_current_pose = m_node_handle.subscribe( "/current_pose", 1, 
     &LowLevelControllerNode::receiveCurrentPoseCallback, this, ros::TransportHints().tcpNoDelay(true));


  m_sub_dbw_enabled = m_node_handle.subscribe( "/vehicle/dbw_enabled", 1, 
     &LowLevelControllerNode::receiveDbwEnabledCallback, this, ros::TransportHints().tcpNoDelay(true));





}

void LowLevelControllerNode::process() {

  m_throttle_pub.publish(m_throttle_cmd);
  m_steer_pub.publish(m_steering_cmd);
  m_brake_pub.publish(m_brake_cmd);
  //  cout << "Low Level Controller - process" << endl;
}


void LowLevelControllerNode::receiveFinalWaypointsCallback(const styx_msgs::Lane &msg) {
  m_final_waypoints = msg;
}

void LowLevelControllerNode::receiveCurrentVelocityCallback(const geometry_msgs::TwistStamped &msg) {
  m_current_velocity = msg;
}

void LowLevelControllerNode::receiveCurrentPoseCallback(const geometry_msgs::PoseStamped &msg) {
  m_current_pose = msg;
}

void LowLevelControllerNode::receiveDbwEnabledCallback(const std_msgs::Bool &msg) {
  m_dbw_enabled = msg.data;
}

