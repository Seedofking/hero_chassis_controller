//
// Created by seed on 2026/11/4.
//

// This project
#include <hero_chassis_controller/hero_chassis_controller.h>

// C++ std depends
#include <array>
#include <vector>

// ROS depends
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
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
  for (int joints_index = 0; joints_index < 4; joints_index++)
  {
    try
    {
      joints_.push_back(hw->getHandle(joints_name[joints_index]));
    }
    catch (const hardware_interface::HardwareInterfaceException e)
    {
      ROS_ERROR_STREAM("Failed to get joint handle for joint "
          << joints_name[joints_index] <<"( " << e.what() << " )");
      return false;
    }
  }

  // Init: Get params from controllers.yaml for pid init
  for (int joints_index = 0; joints_index < 4; joints_index++)
  {
    if (!pids_[joints_index].init(ros::NodeHandle(controller_nh, "wheel/" + joints_name[joints_index])))
    {
      ROS_ERROR_STREAM("Failed to let pid init for joint "
          << joints_name[joints_index] << "(namespace: " << controller_nh.getNamespace() << ")");
      return false;
    }
  }
}

//Update
void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  //TODO: Read from RT
  //TODO: mecanum swift
  std::array<double, 4> target_velocities{ 0 };
  // Update: Turn velocity target to effort target with pid
  for (int joints_index = 0; joints_index < 4; joints_index++)
  {
    const double target_velocity = target_velocities[joints_index];
    const double measured_velocity = joints_[joints_index].getVelocity();
    const double error = target_velocity - measured_velocity;
    joints_[joints_index].setCommand(pids_[joints_index].computeCommand(error, period));
  }
}

void HeroChassisController::starting(const ros::Time& time)
{
}

void HeroChassisController::cmdVelCallback(geometry_msgs::Twist::ConstPtr msg)
{
}
}