//
// Created by seed on 2026/11/4.
//

// This project
#include <hero_chassis_controller/hero_chassis_controller.h>

// C++ std
#include <array>

// ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pluginlib/class_list_macros.h>

namespace hero_chassis_controller
{
bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
}

void HeroChassisController::starting(const ros::Time& time)
{
}

void HeroChassisController::cmdVelCallback(geometry_msgs::Twist::ConstPtr msg)
{
}
}