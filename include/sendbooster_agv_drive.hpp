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

#ifndef SENDBOOSTER_AGV_SIMULATION__SENDBOOSTER_AGV_DRIVE_HPP_
#define SENDBOOSTER_AGV_SIMULATION__SENDBOOSTER_AGV_DRIVE_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <array>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// Scan data indices
enum ScanDirection {
  CENTER = 0,
  LEFT = 1,
  RIGHT = 2,
  SCAN_DATA_SIZE = 3
};

// Robot states
enum RobotState {
  GET_DIRECTION = 0,
  DRIVE_FORWARD = 1,
  TURN_RIGHT = 2,
  TURN_LEFT = 3
};

class SendboosterAgvDrive : public rclcpp::Node
{
public:
  SendboosterAgvDrive();
  ~SendboosterAgvDrive();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables
  double robot_pose_;
  double prev_robot_pose_;
  std::array<double, SCAN_DATA_SIZE> scan_data_;

  // Parameters
  double linear_velocity_;
  double angular_velocity_;
  double forward_distance_;
  double side_distance_;
  double escape_angle_;
  int update_rate_ms_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void declare_parameters();
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif  // SENDBOOSTER_AGV_SIMULATION__SENDBOOSTER_AGV_DRIVE_HPP_
