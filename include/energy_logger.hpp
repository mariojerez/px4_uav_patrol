/****************************************************************************
 * Copyright (c) 2025 Mario Jerez
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <cmath>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

class EnergyLogger
{
public:
  explicit EnergyLogger(rclcpp::Node & node)
  : node_(node)
  {
    battery_sub_ = node_.create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status_v1",
      rclcpp::SensorDataQoS(),
      std::bind(&EnergyLogger::batteryCallback, this, std::placeholders::_1));

    pos_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1",
      rclcpp::SensorDataQoS(),
      std::bind(&EnergyLogger::positionCallback, this, std::placeholders::_1));
  }

  void start()
  {
    energy_j_ = 0.0;
    distance_m_ = 0.0;

    last_timestamp_us_ = 0;
    have_last_pos_ = false;

    active_ = false;  // gated by mission progress
    start_time_ = node_.now();
  }

  void stop()
  {
    active_ = false;
    end_time_ = node_.now();
  }

  void enable()  { active_ = true; }
  void disable() { active_ = false; }

  double energyJoules() const { return energy_j_; }
  double distanceMeters() const { return distance_m_; }

  double durationSeconds() const
  {
    if (end_time_ <= start_time_) return 0.0;
    return (end_time_ - start_time_).seconds();
  }

private:
  /* ---------------- Battery integration ---------------- */

  void batteryCallback(const px4_msgs::msg::BatteryStatus & msg)
  {
    if (!active_) return;

    if (last_timestamp_us_ == 0) {
      last_timestamp_us_ = msg.timestamp;
      return;
    }

    double dt = (msg.timestamp - last_timestamp_us_) * 1e-6; // us → s
    last_timestamp_us_ = msg.timestamp;

    double current = std::abs(msg.current_a);
    double power_w = msg.voltage_v * current;

    energy_j_ += power_w * dt;
  }

  /* ---------------- Distance integration ---------------- */

  void positionCallback(const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    if (!active_ || !msg.xy_valid) return;

    if (!have_last_pos_) {
      last_x_ = msg.x;
      last_y_ = msg.y;
      have_last_pos_ = true;
      return;
    }

    double dx = msg.x - last_x_;
    double dy = msg.y - last_y_;

    distance_m_ += std::sqrt(dx * dx + dy * dy);

    last_x_ = msg.x;
    last_y_ = msg.y;
  }

  /* ---------------- ROS ---------------- */

  rclcpp::Node & node_;

  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;

  /* ---------------- State ---------------- */

  bool active_{false};

  uint64_t last_timestamp_us_{0};
  double energy_j_{0.0};

  bool have_last_pos_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  double distance_m_{0.0};

  rclcpp::Time start_time_;
  rclcpp::Time end_time_;
};
