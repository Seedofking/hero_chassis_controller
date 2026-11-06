//
// Created by seed on 2026/11/4.
//

// This project
#include <hero_chassis_controller/hero_chassis_controller.h>

// C++ std depends
#include <array>
#include <vector>
#include <string>

// ROS depends
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>

namespace hero_chassis_controller
{
//Init
bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Init: Get params from controllers.yaml for basic values and mode
  if (!controller_nh.getParam("publish_rate", publish_rate_) ||
      !controller_nh.getParam("timeout", timeout_) ||
      !controller_nh.getParam("frame_mode", frame_mode_) || //TODO: base and world
      !controller_nh.getParam("wheel_radius", wheel_radius_) ||
      !controller_nh.getParam("wheel_base", wheel_base_) ||
      !controller_nh.getParam("wheel_track", wheel_track_))
  {
    ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

  // Init: Get JointHandle from controllers.yaml for joints
  std::vector<std::string> joints_name;
  if (!controller_nh.getParam("joints", joints_name))
  {
    ROS_ERROR("Can't load joints_name (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (joints_name.size() != 4)
  {
    ROS_ERROR("The number of joints isn't 4 (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  for (size_t joints_index = 0; joints_index < 4; joints_index++)
  {
    try
    {
      joints_.push_back(hw->getHandle(joints_name[joints_index]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Failed to get joint handle for joint "
          << joints_name[joints_index] <<"( " << e.what() << " )");
      return false;
    }
  }

  // Init: Get params from controllers.yaml for pid init
  for (size_t joints_index = 0; joints_index < 4; joints_index++)
  {
    if (!pids_[joints_index].init(ros::NodeHandle(controller_nh, "wheel/" + joints_name[joints_index])))
    {
      ROS_ERROR_STREAM("Failed to let pid init for joint "
          << joints_name[joints_index] << "(namespace: " << controller_nh.getNamespace() << ")");
      return false;
    }
  }

  // Init: Initialize odometry publisher, tf broadcaster, cmd_vel subscriber
  odom_pub_ = std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>
      (root_nh, "odom", 100);
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = "base_link";
  tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist::ConstPtr>
      ("cmd_vel", 100, &HeroChassisController::cmdVelCallback, this);

  return true;
}

//Update
void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // Update: Read cmd form RT
  geometry_msgs::Twist cmd_vel = *cmd_vel_buffer_.readFromRT();

  // Update: Timeout protect
  if ((time - last_cmd_time_).toSec() > timeout_) //TODO: Who is last_cmd_time_?
  {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
  }

  // Update: Mecanum wheel inverse kinematics
  std::array<double, 4> target_wheels_velocity{};
  const double L = (wheel_base_ + wheel_track_) / 2;

  target_wheels_velocity[kLeftFront] = (cmd_vel.linear.x - cmd_vel.linear.y - cmd_vel.angular.z * L) / wheel_radius_;
  target_wheels_velocity[kRightFront] = (cmd_vel.linear.x + cmd_vel.linear.y + cmd_vel.angular.z * L) / wheel_radius_;
  target_wheels_velocity[kLeftBack] = (cmd_vel.linear.x + cmd_vel.linear.y - cmd_vel.angular.z * L) / wheel_radius_;
  target_wheels_velocity[kRightBack] = (cmd_vel.linear.x - cmd_vel.linear.y + cmd_vel.angular.z * L) / wheel_radius_;

  // Update: Turn velocity target to effort target with pid
  for (int joints_index = 0; joints_index < 4; joints_index++)
  {
    const double target_wheel_velocity = target_wheels_velocity[joints_index];
    const double measured_wheel_velocity = joints_[joints_index].getVelocity();
    const double error = target_wheel_velocity - measured_wheel_velocity;
    joints_[joints_index].setCommand(pids_[joints_index].computeCommand(error, period));
  }
}

void HeroChassisController::starting(const ros::Time& time)
{
  // Starting : Initialize time
  last_cmd_time_ = ros::Time::now();
  last_publish_time_ = ros::Time::now();
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr msg)
{
  cmd_vel_buffer_.writeFromNonRT(*msg);
  last_cmd_time_ = ros::Time::now();
}
} // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase);