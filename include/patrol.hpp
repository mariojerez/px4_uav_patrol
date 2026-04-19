/****************************************************************************
 * Copyright (c) 2025 Mario Jerez
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

#include "energy_logger.hpp"
#include "patrol_logic.hpp"


static const std::string kName = "Patrol Mission";

// Patrol wraps a px4_ros2 MissionExecutor so it can be driven from an action
// server: construction is cheap (no mission loaded) and the mission lifecycle
// is managed by explicit start()/cancel() calls. The single registered
// MissionExecutor is reused across successive start() calls by swapping the
// mission via setMission()/resetMission().
class Patrol
{
public:
  explicit Patrol(const std::shared_ptr<rclcpp::Node> & node)
  : _node(node)
  {
    mission_id_default_ =
      node->declare_parameter<std::string>("mission_id", "unknown");

    energy_logger_ = std::make_unique<EnergyLogger>(*node);
    energy_logger_->start();
  }

  // Load the mission file and kick off execution. Returns false with an
  // error description in `error_msg` if the file cannot be loaded or does
  // not contain any navigation waypoints.
  //
  // The first call lazily constructs and registers the px4_ros2
  // MissionExecutor; subsequent calls re-use the existing registration.
  bool start(const std::string & mission_file, std::string & error_msg)
  {
    if (mission_file.empty()) {
      error_msg = "mission_file is empty";
      return false;
    }
    if (active_) {
      error_msg = "patrol already active; cancel before starting again";
      return false;
    }

    std::ifstream f(mission_file);
    if (!f.is_open()) {
      error_msg = "Failed to open mission file: " + mission_file;
      return false;
    }
    std::string json_text(
      (std::istreambuf_iterator<char>(f)),
      std::istreambuf_iterator<char>());

    nlohmann::json mission_json;
    try {
      mission_json = nlohmann::json::parse(json_text);
    } catch (const std::exception & e) {
      error_msg = std::string("Mission JSON parse error: ") + e.what();
      return false;
    }

    target_groundspeed_mps_ = 0.0;
    if (mission_json["mission"].contains("defaults") &&
        mission_json["mission"]["defaults"].contains("horizontalVelocity"))
    {
      target_groundspeed_mps_ =
        mission_json["mission"]["defaults"]["horizontalVelocity"].get<double>();
    }

    first_wp_idx_ = -1;
    last_wp_idx_ = -1;
    const auto & items = mission_json["mission"]["items"];
    for (size_t i = 0; i < items.size(); ++i) {
      if (items[i].value("type", "") == "navigation" &&
          items[i].value("navigationType", "") == "waypoint")
      {
        if (first_wp_idx_ < 0) {
          first_wp_idx_ = static_cast<int>(i);
        }
        last_wp_idx_ = static_cast<int>(i);
      }
      if (items[i].value("type", "") == "changeSettings" &&
          items[i].contains("horizontalVelocity"))
      {
        target_groundspeed_mps_ =
          items[i]["horizontalVelocity"].get<double>();
      }
    }
    if (first_wp_idx_ < 0) {
      error_msg = "Mission contains no navigation waypoints";
      return false;
    }

    mission_id_ = mission_json.value("mission_id", mission_id_default_);
    current_index_ = -1;
    completed_ = false;
    aborted_ = false;

    energy_logger_->start();

    auto mission = px4_ros2::Mission(mission_json);

    if (!_mission_executor) {
      _mission_executor = std::make_unique<px4_ros2::MissionExecutor>(
        kName,
        px4_ros2::MissionExecutor::Configuration(),
        *_node);

      if (!_mission_executor->doRegister()) {
        _mission_executor.reset();
        error_msg = "Failed to register mission executor";
        return false;
      }

      _mission_executor->onProgressUpdate(
        [this](int current_index) {
          current_index_ = current_index;
          if (current_index == first_wp_idx_) {
            RCLCPP_INFO(
              _node->get_logger(),
              "Entering waypoint traversal (target speed %.2f m/s)",
              target_groundspeed_mps_);
            energy_logger_->enable();
          }
          if (current_index > last_wp_idx_) {
            RCLCPP_INFO(_node->get_logger(), "Exiting waypoint traversal");
            energy_logger_->disable();
          }
        });

      _mission_executor->onCompleted(
        [this]() {
          energy_logger_->disable();
          energy_logger_->stop();
          completion_time_utc_s_ =
            std::chrono::duration<double>(
              std::chrono::system_clock::now().time_since_epoch()).count();
          writeCsv();
          completed_ = true;
          active_ = false;
        });
    }

    _mission_executor->setMission(mission);
    active_ = true;
    return true;
  }

  // Preempt the currently running mission. After abort() the executor will
  // fall into its onFailure action (land/hold) until the next start().
  void cancel()
  {
    if (!active_ || !_mission_executor) {
      return;
    }
    RCLCPP_INFO(_node->get_logger(), "Patrol cancel requested; aborting mission");
    _mission_executor->abort();
    energy_logger_->disable();
    energy_logger_->stop();
    aborted_ = true;
    active_ = false;
  }

  bool isActive()        const { return active_; }
  bool isComplete()      const { return completed_; }
  bool wasAborted()      const { return aborted_; }
  int  currentWaypoint() const { return current_index_; }

  float progressPercent() const
  {
    return patrol_logic::progressPercent(current_index_, first_wp_idx_, last_wp_idx_);
  }

  double energyJoules()    const { return energy_logger_->energyJoules(); }
  double distanceMeters()  const { return energy_logger_->distanceMeters(); }

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

  std::shared_ptr<rclcpp::Node> _node;
  std::unique_ptr<px4_ros2::MissionExecutor> _mission_executor;
  std::unique_ptr<EnergyLogger> energy_logger_;

  std::string mission_id_default_;
  std::string mission_id_;

  int first_wp_idx_{-1};
  int last_wp_idx_{-1};
  int current_index_{-1};

  double target_groundspeed_mps_{0.0};
  double completion_time_utc_s_{0.0};

  bool active_{false};
  bool completed_{false};
  bool aborted_{false};
};
