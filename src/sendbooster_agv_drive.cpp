// Copyright 2024 TerraNox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: 이성민 (roboticsmaster@naver.com)

#include "sendbooster_agv_drive.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

SendboosterAgvDrive::SendboosterAgvDrive()
: Node("sendbooster_agv_drive_node")
{
  /************************************************************
  ** Declare and get parameters
  ************************************************************/
  declare_parameters();

  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_.fill(0.0);
  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan",
    rclcpp::SensorDataQoS(),
    std::bind(&SendboosterAgvDrive::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos,
    std::bind(&SendboosterAgvDrive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  auto timer_period = std::chrono::milliseconds(update_rate_ms_);
  update_timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&SendboosterAgvDrive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Sendbooster AGV drive node initialized");
  RCLCPP_INFO(this->get_logger(), "Parameters - linear_vel: %.2f, angular_vel: %.2f, "
    "forward_dist: %.2f, side_dist: %.2f, escape_angle: %.1f deg, update_rate: %d ms",
    linear_velocity_, angular_velocity_, forward_distance_, side_distance_,
    escape_angle_ * RAD2DEG, update_rate_ms_);
}

SendboosterAgvDrive::~SendboosterAgvDrive()
{
  RCLCPP_INFO(this->get_logger(), "Sendbooster AGV drive node terminated");
}

void SendboosterAgvDrive::declare_parameters()
{
  // Declare parameters with default values
  this->declare_parameter<double>("linear_velocity", 0.3);
  this->declare_parameter<double>("angular_velocity", 1.5);
  this->declare_parameter<double>("forward_distance", 0.7);
  this->declare_parameter<double>("side_distance", 0.6);
  this->declare_parameter<double>("escape_angle", 30.0);
  this->declare_parameter<int>("update_rate_ms", 20);

  // Get parameter values
  linear_velocity_ = this->get_parameter("linear_velocity").as_double();
  angular_velocity_ = this->get_parameter("angular_velocity").as_double();
  forward_distance_ = this->get_parameter("forward_distance").as_double();
  side_distance_ = this->get_parameter("side_distance").as_double();
  escape_angle_ = this->get_parameter("escape_angle").as_double() * DEG2RAD;
  update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();

  // Validate parameters
  if (linear_velocity_ < 0.0 || linear_velocity_ > 2.0) {
    RCLCPP_WARN(this->get_logger(), "linear_velocity out of range, clamping to [0.0, 2.0]");
    linear_velocity_ = std::clamp(linear_velocity_, 0.0, 2.0);
  }
  if (angular_velocity_ < 0.0 || angular_velocity_ > 3.0) {
    RCLCPP_WARN(this->get_logger(), "angular_velocity out of range, clamping to [0.0, 3.0]");
    angular_velocity_ = std::clamp(angular_velocity_, 0.0, 3.0);
  }
  if (update_rate_ms_ < 10 || update_rate_ms_ > 1000) {
    RCLCPP_WARN(this->get_logger(), "update_rate_ms out of range, clamping to [10, 1000]");
    update_rate_ms_ = std::clamp(update_rate_ms_, 10, 1000);
  }
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void SendboosterAgvDrive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void SendboosterAgvDrive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Scan angles: 0 (front), 30 (left-front), 330 (right-front)
  constexpr std::array<uint16_t, SCAN_DATA_SIZE> scan_angles = {0, 30, 330};

  for (size_t i = 0; i < SCAN_DATA_SIZE; i++) {
    // Bounds checking
    if (scan_angles[i] < msg->ranges.size()) {
      double range = msg->ranges[scan_angles[i]];
      // Handle infinity and NaN values
      if (std::isinf(range) || std::isnan(range)) {
        scan_data_[i] = msg->range_max;
      } else {
        scan_data_[i] = std::clamp(range,
          static_cast<double>(msg->range_min),
          static_cast<double>(msg->range_max));
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Scan angle %d out of range (size: %zu)", scan_angles[i], msg->ranges.size());
      scan_data_[i] = msg->range_max;
    }
  }
}

void SendboosterAgvDrive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void SendboosterAgvDrive::update_callback()
{
  static uint8_t robot_state = GET_DIRECTION;

  switch (robot_state) {
    case GET_DIRECTION:
      if (scan_data_[CENTER] > forward_distance_) {
        if (scan_data_[LEFT] < side_distance_) {
          prev_robot_pose_ = robot_pose_;
          robot_state = TURN_RIGHT;
        } else if (scan_data_[RIGHT] < side_distance_) {
          prev_robot_pose_ = robot_pose_;
          robot_state = TURN_LEFT;
        } else {
          robot_state = DRIVE_FORWARD;
        }
      } else {
        prev_robot_pose_ = robot_pose_;
        robot_state = TURN_RIGHT;
      }
      break;

    case DRIVE_FORWARD:
      update_cmd_vel(linear_velocity_, 0.0);
      robot_state = GET_DIRECTION;
      break;

    case TURN_RIGHT:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_angle_) {
        robot_state = GET_DIRECTION;
      } else {
        update_cmd_vel(0.0, -angular_velocity_);
      }
      break;

    case TURN_LEFT:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_angle_) {
        robot_state = GET_DIRECTION;
      } else {
        update_cmd_vel(0.0, angular_velocity_);
      }
      break;

    default:
      robot_state = GET_DIRECTION;
      break;
  }
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendboosterAgvDrive>());
  rclcpp::shutdown();

  return 0;
}
