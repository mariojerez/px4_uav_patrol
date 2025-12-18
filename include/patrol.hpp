/****************************************************************************
 * Copyright (c) 2025 Mario Jerez
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <fstream>
#include <iterator>
#include <filesystem>
#include <chrono>

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

#include "energy_logger.hpp"


static const std::string kName = "Patrol Mission";

class Patrol
{
public:
  explicit Patrol(const std::shared_ptr<rclcpp::Node> & node)
  : _node(node)
  {
    /* ---------- Parameters ---------- */

    mission_id_ =
      node->declare_parameter<std::string>("mission_id", "unknown");

    std::string mission_file =
      node->declare_parameter<std::string>("mission_file", "");

    if (mission_file.empty()) {
      throw std::runtime_error("Parameter 'mission_file' is empty");
    }

    /* ---------- Load mission JSON ---------- */

    std::ifstream f(mission_file);
    if (!f.is_open()) {
      throw std::runtime_error("Failed to open mission file: " + mission_file);
    }

    std::string json_text(
      (std::istreambuf_iterator<char>(f)),
       std::istreambuf_iterator<char>());

    nlohmann::json mission_json = nlohmann::json::parse(json_text);

    /* ---------- Extract target groundspeed ---------- */

    target_groundspeed_mps_ = 0.0;

    if (mission_json["mission"].contains("defaults") &&
        mission_json["mission"]["defaults"].contains("horizontalVelocity")) {

      target_groundspeed_mps_ =
        mission_json["mission"]["defaults"]["horizontalVelocity"].get<double>();
    }

    /* ---------- Identify waypoint range ---------- */

    const auto & items = mission_json["mission"]["items"];

    for (size_t i = 0; i < items.size(); ++i) {

      if (items[i].value("type", "") == "navigation" &&
          items[i].value("navigationType", "") == "waypoint") {

        if (first_wp_idx_ < 0) {
          first_wp_idx_ = static_cast<int>(i);
        }
        last_wp_idx_ = static_cast<int>(i);
      }

      if (items[i].value("type", "") == "changeSettings" &&
          items[i].contains("horizontalVelocity")) {

        target_groundspeed_mps_ =
          items[i]["horizontalVelocity"].get<double>();
      }
    }

    if (first_wp_idx_ < 0) {
      throw std::runtime_error("Mission contains no navigation waypoints");
    }

    /* ---------- Energy logger ---------- */

    energy_logger_ = std::make_unique<EnergyLogger>(*node);
    energy_logger_->start();

    /* ---------- Mission executor ---------- */

    auto mission = px4_ros2::Mission(mission_json);

    _mission_executor = std::make_unique<px4_ros2::MissionExecutor>(
      kName,
      px4_ros2::MissionExecutor::Configuration(),
      *node);


    if (!_mission_executor->doRegister()) {
      throw std::runtime_error("Failed to register mission executor");
    }

    _mission_executor->setMission(mission);

    /* ---------- Gate logging by mission index ---------- */

    _mission_executor->onProgressUpdate(
      [&](int current_index) {

        if (current_index == first_wp_idx_) {
          RCLCPP_INFO(
            _node->get_logger(),
            "Entering waypoint traversal (target speed %.2f m/s)",
            target_groundspeed_mps_);
          energy_logger_->enable();
        }

        if (current_index > last_wp_idx_) {
          RCLCPP_INFO(_node->get_logger(),
                      "Exiting waypoint traversal");
          energy_logger_->disable();
        }
      });

    /* ---------- Completion ---------- */

    _mission_executor->onCompleted([&]() {

      energy_logger_->disable();
      energy_logger_->stop();

      completion_time_utc_s_ =
        std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch()
        ).count();

      writeCsv();
    });
  }

private:
  void writeCsv()
  {
    bool write_header = !std::filesystem::exists("energy_log.csv");
    std::ofstream f("energy_log.csv", std::ios::app);

    if (write_header) {
      f << "mission,"
           "target_groundspeed_mps,"
           "energy_joules,"
           "distance_m,"
           "energy_j_per_m,"
           "traversal_duration_s,"
           "completion_time_utc_s\n";
    }

    double energy = energy_logger_->energyJoules();
    double distance = energy_logger_->distanceMeters();
    double j_per_m = (distance > 1e-3) ? (energy / distance) : 0.0;

    f << mission_id_ << ","
      << target_groundspeed_mps_ << ","
      << energy << ","
      << distance << ","
      << j_per_m << ","
      << energy_logger_->durationSeconds() << ","
      << completion_time_utc_s_ << "\n";
  }

  /* ---------- Members ---------- */

  std::shared_ptr<rclcpp::Node> _node;
  std::unique_ptr<px4_ros2::MissionExecutor> _mission_executor;
  std::unique_ptr<EnergyLogger> energy_logger_;

  std::string mission_id_;

  int first_wp_idx_{-1};
  int last_wp_idx_{-1};

  double target_groundspeed_mps_{0.0};
  double completion_time_utc_s_{0.0};
};
