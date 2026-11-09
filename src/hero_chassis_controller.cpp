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
#include <angles/angles.h>

namespace hero_chassis_controller
{
//Init
bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Init: Get params from controllers.yaml for basic values and mode
  if (!controller_nh.getParam("publish_rate", publish_rate_) ||
      !controller_nh.getParam("timeout", timeout_) ||
      !controller_nh.getParam("frame_mode", frame_mode_) ||
      !controller_nh.getParam("use_velocity_ramp", use_velocity_ramp_) ||
      !controller_nh.getParam("velocity_ramp_step", velocity_ramp_step_) ||
      !controller_nh.getParam("wheel_radius", wheel_radius_))
  {
    ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

  // Init: Joints init
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
    // Get JointHandle from controllers.yaml for joints
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

    // Get params from controllers.yaml for pid init and wheel pose
    std::string wheel_prefix = "wheel/" + joints_name[joints_index];
    XmlRpc::XmlRpcValue pose;
    if (!pids_[joints_index].init(ros::NodeHandle(controller_nh, wheel_prefix + "/pid")))
    {
      ROS_ERROR_STREAM("Failed to let pid init for joint "
          << joints_name[joints_index] << "(namespace: " << controller_nh.getNamespace() << ")");
      return false;
    }
    if (!controller_nh.getParam(wheel_prefix + "/pose", pose))
    {
      ROS_ERROR_STREAM("Failed to let pose init for joint "
          << joints_name[joints_index] << "(namespace: " << controller_nh.getNamespace() << ")");
      return false;
    }
    poses_[joints_index][kPoseX] = pose["x"];
    poses_[joints_index][kPoseY] = pose["y"];
    poses_[joints_index][kPoseRollerAngle] = pose["roller_angle"];

    // Set matrix for kinematics
    Eigen::Matrix<double, 1, 2> in_roller_matrix{};
    Eigen::Matrix<double, 2, 3> in_chassis_matrix{};
    in_roller_matrix << 1, tan(static_cast<double>(pose["roller_angle"]));
    in_chassis_matrix << 1, 0, -static_cast<double>(pose["y"]),
        0, 1, static_cast<double>(pose["x"]);

    Eigen::Matrix<double, 1, 3> chassis2joint_matrix = in_roller_matrix * in_chassis_matrix * (1 / wheel_radius_);
    jacobian_.row(static_cast<long>(joints_index)) = chassis2joint_matrix;
  }

  // Init: Initialize odometry publisher, tf broadcaster, cmd_vel subscriber
  odom_pub_ = std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>
      (root_nh, "odom", 100);
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = "base_link";

  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
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
  if ((time - last_cmd_time_).toSec() > timeout_)
  {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
  }

  // Update: Transform cmd_vel to world under frame_mode_ = "world"
  if (frame_mode_ == "world")
  {
    try
    {
      geometry_msgs::TransformStamped tf =
          tf2_buffer_->lookupTransform("base_link", "odom", ros::Time(0));
      geometry_msgs::Vector3 cmd_vel_input, cmd_vel_output;
      cmd_vel_input.x = cmd_vel.linear.x;
      cmd_vel_input.y = cmd_vel.linear.y;
      cmd_vel_input.z = cmd_vel.angular.z;
      tf2::doTransform(cmd_vel_input, cmd_vel_output, tf);
      cmd_vel.linear.x = cmd_vel_output.x;
      cmd_vel.linear.y = cmd_vel_output.y;
      cmd_vel.angular.z = cmd_vel_output.z;
    }
    catch (const tf2::TransformException& ex)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.angular.z = 0;
    }
  }

  // Update: Velocity ramp
  if (use_velocity_ramp_)
  {
    std::array<double, 3> cmd_vel_input{ cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z };
    double step = velocity_ramp_step_;
    static std::array<double, 3> last_cmd_vel_output{};
    for (size_t vel_index = 0; vel_index < 3; vel_index++)
    {
      if (vel_index == 2)
      {
        step *= 2;
      }
      if (cmd_vel_input[vel_index] > last_cmd_vel_output[vel_index])
      {
        double vel_out_plus_step = last_cmd_vel_output[vel_index] + step;
        last_cmd_vel_output[vel_index] =
            cmd_vel_input[vel_index] >= vel_out_plus_step ? vel_out_plus_step : cmd_vel_input[vel_index];
      }
      else if (cmd_vel_input[vel_index] < last_cmd_vel_output[vel_index])
      {
        double vel_out_minus_step = last_cmd_vel_output[vel_index] - step;
        last_cmd_vel_output[vel_index] =
            cmd_vel_input[vel_index] <= vel_out_minus_step ? vel_out_minus_step : cmd_vel_input[vel_index];
      }
      else
      {
        last_cmd_vel_output[vel_index] = cmd_vel_input[vel_index];
      }
    }
    cmd_vel.linear.x = last_cmd_vel_output[0];
    cmd_vel.linear.y = last_cmd_vel_output[1];
    cmd_vel.angular.z = last_cmd_vel_output[2];
  }

  // Update: Mecanum wheel inverse kinematics
  Eigen::Vector4d target_wheels_velocity{};
  Eigen::Vector3d cmd_chassis_velocity{};
  std::array<double, 4> measured_wheels_velocity{};

  cmd_chassis_velocity << cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z;
  target_wheels_velocity = jacobian_ * cmd_chassis_velocity;

  // Update: Max wheel velocity limitation
  if (target_wheels_velocity.maxCoeff() > 50)
  {
    target_wheels_velocity *= 50 / target_wheels_velocity.maxCoeff();
  }

  // Update: Turn target velocity to target effort using pid
  for (int joints_index = 0; joints_index < 4; joints_index++)
  {
    measured_wheels_velocity[joints_index] = joints_[joints_index].getVelocity();
    const double error = target_wheels_velocity[joints_index] - measured_wheels_velocity[joints_index];
    joints_[joints_index].setCommand(pids_[joints_index].computeCommand(error, period));
  }

  // Update: Turn measured velocity to odometry
  Eigen::Matrix<double, 4, 1> measured_wheels_velocity_matrix{};
  Eigen::Matrix<double, 3, 1> base_velocity_matrix{};
  base_velocity_matrix.setZero();
  measured_wheels_velocity_matrix <<
      measured_wheels_velocity[kLeftFront],
      measured_wheels_velocity[kRightFront],
      measured_wheels_velocity[kLeftBack],
      measured_wheels_velocity[kRightBack];

  base_velocity_matrix =
      (jacobian_.transpose() * jacobian_).inverse() * jacobian_.transpose() * measured_wheels_velocity_matrix;
  double base_vx = base_velocity_matrix[0];
  double base_vy = base_velocity_matrix[1];
  double base_wz = base_velocity_matrix[2];

  const double dt = period.toSec();
  const double odom_wz = base_wz;
  const double d_theta = odom_wz * dt;
  const double mid_theta = odom_theta_ + d_theta / 2;
  odom_x_ += (base_vx * std::cos(mid_theta) - base_vy * std::sin(mid_theta)) * dt;
  odom_y_ += (base_vx * std::sin(mid_theta) + base_vy * std::cos(mid_theta)) * dt;
  odom_theta_ = angles::normalize_angle(odom_theta_ + d_theta);

  // Update: Publish
  if ((time - last_publish_time_).toSec() > 1.0 / publish_rate_)
  {
    // Update: Publish odometry and twist
    if (odom_pub_->trylock())
    {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.header.frame_id = "odom";
      odom_pub_->msg_.child_frame_id = "base_link";

      odom_pub_->msg_.pose.pose.position.x = odom_x_;
      odom_pub_->msg_.pose.pose.position.y = odom_y_;

      odom_pub_->msg_.pose.pose.orientation.z = sin(odom_theta_ / 2);
      odom_pub_->msg_.pose.pose.orientation.w = cos(odom_theta_ / 2);

      odom_pub_->msg_.twist.twist.linear.x = base_vx;
      odom_pub_->msg_.twist.twist.linear.y = base_vy;
      odom_pub_->msg_.twist.twist.angular.z = base_wz;

      odom_pub_->unlockAndPublish();
    }

    // Update: Broadcast transform
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.rotation.z = sin(odom_theta_ / 2);
    tf.transform.rotation.w = cos(odom_theta_ / 2);
    tf2_broadcaster_->sendTransform(tf);
    last_publish_time_ = time;
  }
}

void HeroChassisController::starting(const ros::Time& time)
{
  // Starting : Initialize time
  last_cmd_time_ = ros::Time::now();
  last_publish_time_ = ros::Time::now();

  for (size_t joints_index = 0; joints_index < 4; joints_index++)
  {
    pids_[joints_index].reset();
  }
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr msg)
{
  cmd_vel_buffer_.writeFromNonRT(*msg);
  last_cmd_time_ = ros::Time::now();
}
} // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase);