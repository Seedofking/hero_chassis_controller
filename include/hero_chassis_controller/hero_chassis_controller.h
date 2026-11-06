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
//#include <Eigen/Dense>  TODO: Turn to matrix
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
  //Eigen::Matrix<double, 4, 3> jacobian_; TODO: Turn to matrix
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
  ros::Subscriber cmd_vel_sub_;

  //config
  double publish_rate_{ 100 };
  double timeout_{ 0.1 };
  std::string frame_mode_{ "base" };
  double wheel_radius_{ 0.07625 }; // From rm_description_for_task/urdf/common/mecanum_wheel.urdf.xacro
  double wheel_base_{ 0.4 };       // From rm_description_for_task/urdf/hero/chassis.urdf.xacro
  double wheel_track_{ 0.4 };      // From rm_description_for_task/urdf/hero/chassis.urdf.xacro
  enum WheelIndex : size_t
  {
    kLeftFront = 0,
    kRightFront = 1,
    kLeftBack = 2,
    kRightBack = 3
  };


  //state
  ros::Time last_cmd_time_;
  ros::Time last_publish_time_;
  double odom_x_{ 0.0 };
  double odom_y_{ 0.0 };
  double odom_theta_{ 0.0 };
};
}