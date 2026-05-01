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

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <px4_ros2/utils/vehicle_command_sender.hpp>

#include "energy_logger.hpp"
#include "patrol_logic.hpp"


static const std::string kName = "Patrol Mission";

// MissionExecutor::modeExecutor() and modeId() are protected. The auto-arm
// path needs both (to send VEHICLE_CMD_COMPONENT_ARM_DISARM and switch into
// the dynamically-allocated Patrol custom mode). Subclass and re-publish.
class PatrolMissionExecutor : public px4_ros2::MissionExecutor
{
public:
  using px4_ros2::MissionExecutor::MissionExecutor;
  using px4_ros2::MissionExecutor::modeExecutor;
  using px4_ros2::MissionExecutor::modeId;
};

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

    _command_sender = std::make_unique<px4_ros2::VehicleCommandSender>(*node, "");
  }

  // Load the mission file and kick off execution. Returns false with an
  // error description in `error_msg` if the file cannot be loaded or does
  // not contain any navigation waypoints.
  //
  // `per_leg_meta_file`: optional path to a per-leg metadata sidecar JSON.
  //   When non-empty, the EnergyLogger emits one CSV row per leg defined in
  //   that file (energy-calibration mode); when empty, falls back to the
  //   legacy single-row aggregate output to energy_log.csv.
  //
  // `auto_arm`: when true, the patrol_mission node arms the FMU and switches
  //   into the registered Patrol custom mode itself (via the px4_ros2
  //   ModeExecutorBase API), so the caller does not need to flip mode and
  //   click Arm in QGC. Used by the calibration runner. The first arm allows
  //   pre-flight checks to run; subsequent arms skip them.
  //
  // The first call lazily constructs and registers the px4_ros2
  // MissionExecutor; subsequent calls re-use the existing registration.
  bool start(
    const std::string & mission_file,
    std::string & error_msg,
    const std::string & per_leg_meta_file = "",
    bool auto_arm = false)
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

    std::unique_ptr<px4_ros2::Mission> mission;
    try {
      mission = std::make_unique<px4_ros2::Mission>(mission_json);
    } catch (const std::exception & e) {
      error_msg = std::string("Mission JSON is invalid: ") + e.what();
      return false;
    }

    energy_logger_->start();

    // Snapshot mode: load per-leg metadata if a sidecar JSON path was given.
    // When loaded, the existing onProgressUpdate callback drives leg
    // start/end transitions in the logger via onProgressTick(), and each
    // completed leg appends a row to the CSV path stored in the meta file.
    if (!per_leg_meta_file.empty()) {
      std::string meta_err;
      if (!energy_logger_->loadLegMetadata(per_leg_meta_file, meta_err)) {
        error_msg = "Failed to load per-leg metadata: " + meta_err;
        return false;
      }
      RCLCPP_INFO(_node->get_logger(),
        "EnergyLogger snapshot mode enabled from %s",
        per_leg_meta_file.c_str());
    } else {
      energy_logger_->clearLegMetadata();
    }

    if (!_mission_executor) {
      _mission_executor = std::make_unique<PatrolMissionExecutor>(
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
          if (energy_logger_->snapshotMode()) {
            // Snapshot mode: leg start/end transitions are driven by the
            // metadata sidecar; do not gate on first/last waypoint.
            energy_logger_->onProgressTick(current_index);
          } else {
            // Legacy mode: log only between the first and last navigation
            // waypoints in the mission JSON.
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
          }
        });

      _mission_executor->onCompleted(
        [this]() {
          energy_logger_->disable();
          energy_logger_->stop();
          completion_time_utc_s_ =
            std::chrono::duration<double>(
              std::chrono::system_clock::now().time_since_epoch()).count();
          // In snapshot mode, per-leg CSV rows have already been appended by
          // the logger; the legacy aggregate CSV would double-count.
          if (!energy_logger_->snapshotMode()) {
            writeCsv();
          }
          completed_ = true;
          active_ = false;
        });
    }

    _mission_executor->setMission(*mission);
    active_ = true;

    if (auto_arm) {
      armAndActivate();
    }
    return true;
  }

  // Async sequence: wait until pre-flight checks pass, arm the vehicle, then
  // switch into our registered Patrol mode. Sets aborted_ if any step fails
  // so the action server tick reaps it. Subsequent arms in the same process
  // skip pre-flight (cuts ~5 s from each calibration trial).
  //
  // Why we bypass me.arm() / me.scheduleMode(): the upstream MissionExecutor
  // hardcodes its mode-executor's Settings to Activation::ActivateOnlyWhenArmed
  // (mission_executor.cpp:58). With that setting, our executor only becomes
  // "in charge" once already armed — but the executor's arm() and
  // scheduleMode() tag their VehicleCommands with
  // source_component = COMPONENT_MODE_EXECUTOR_START + id(), and PX4's
  // commander rejects executor-tagged commands from non-in-charge executors
  // ("Got cmd from executor X not in charge"). The executor cannot arm itself.
  // We work around this by sending VEHICLE_CMD_COMPONENT_ARM_DISARM and
  // VEHICLE_CMD_SET_NAV_STATE via VehicleCommandSender with the default
  // source_component (0); the executor-authority gate only fires for
  // source_component >= 1000, so PX4 accepts these as GCS-style commands.
  // Once armed and in our owned mode, executor_in_charge == us && _is_armed
  // is true, the mode_executor activates, and the mission begins.
  void armAndActivate()
  {
    using px4_ros2::Result;
    auto & me = _mission_executor->modeExecutor();
    const auto patrol_mode_id = _mission_executor->modeId();
    const bool run_preflight = !preflight_already_run_;

    me.waitReadyToArm([this, patrol_mode_id, run_preflight](Result r) {
      if (r != Result::Success) {
        RCLCPP_ERROR(_node->get_logger(),
          "Auto-arm: waitReadyToArm failed (Result=%d)", static_cast<int>(r));
        aborted_ = true;
        active_ = false;
        return;
      }

      px4_msgs::msg::VehicleCommand arm_cmd{};
      arm_cmd.command =
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
      arm_cmd.param1 = 1.0f;
      arm_cmd.param2 = run_preflight ? NAN : 21196.0f;
      arm_cmd.target_system = 1;
      arm_cmd.target_component = 1;
      const Result arm_result = _command_sender->sendCommandSync(arm_cmd);
      if (arm_result != Result::Success) {
        RCLCPP_ERROR(_node->get_logger(),
          "Auto-arm: arm() failed (Result=%d)",
          static_cast<int>(arm_result));
        aborted_ = true;
        active_ = false;
        return;
      }
      preflight_already_run_ = true;

      px4_msgs::msg::VehicleCommand set_mode_cmd{};
      set_mode_cmd.command =
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
      set_mode_cmd.param1 = static_cast<float>(patrol_mode_id);
      set_mode_cmd.target_system = 1;
      set_mode_cmd.target_component = 1;
      const Result mode_result = _command_sender->sendCommandSync(set_mode_cmd);
      if (mode_result != Result::Success) {
        RCLCPP_ERROR(_node->get_logger(),
          "Auto-arm: scheduleMode(Patrol) failed (Result=%d)",
          static_cast<int>(mode_result));
        aborted_ = true;
        active_ = false;
        return;
      }
      RCLCPP_INFO(_node->get_logger(),
        "Auto-arm: armed and Patrol mode active");
    });
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
  std::unique_ptr<PatrolMissionExecutor> _mission_executor;
  std::unique_ptr<EnergyLogger> energy_logger_;
  std::unique_ptr<px4_ros2::VehicleCommandSender> _command_sender;

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
  bool preflight_already_run_{false};
};
