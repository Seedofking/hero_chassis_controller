//
// Created by seed on 2026/11/4.
//

#pragma once

// C++ std
#include <vector>
#include <array>
#include <memory>
#include <string>

// ROS
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <control_toolbox/pid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace hero_chassis_controller
{
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface* hw,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;

private:
  void cmdVelCallback(geometry_msgs::Twist::ConstPtr msg);

  //resource
  std::vector<hardware_interface::JointHandle> joints_;
  std::array<control_toolbox::Pid, 4> pids_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
  Eigen::Matrix<double, 4, 3> jacobian_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
  ros::Subscriber cmd_vel_sub_;

  //config
  double publish_rate_{ 100 };
  double timeout_{ 0.1 };
  std::string frame_mode_{ "base" };
  bool use_velocity_ramp_{ true };
  double velocity_ramp_step_{ 0.002 };
  double wheel_radius_{ 0.07625 }; // From rm_description_for_task/urdf/common/mecanum_wheel.urdf.xacro
  std::array<std::array<double, 3>, 4> poses_{};

  enum WheelIndex : size_t
  {
    kLeftFront = 0,
    kRightFront = 1,
    kLeftBack = 2,
    kRightBack = 3
  };

  enum PoseIndex : size_t
  {
    kPoseX = 0,
    kPoseY = 1,
    kPoseRollerAngle = 2
  };

  //state
  ros::Time last_cmd_time_;
  ros::Time last_publish_time_;
  double odom_x_{ 0.0 };
  double odom_y_{ 0.0 };
  double odom_theta_{ 0.0 };
};
}