/****************************************************************************
 * Copyright (c) 2025 Mario Jerez
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>


// Per-leg snapshot specification loaded from a sidecar metadata JSON. In
// snapshot mode (loadLegMetadata() called) the EnergyLogger emits one CSV row
// per leg instead of a single aggregate row at mission completion. Each leg
// is bounded by mission item indices reported by the px4_ros2 mission
// executor's onProgressUpdate callback: accumulators reset on entering
// `start_idx`, a CSV row is emitted on entering `end_idx`.
struct LegSpec
{
  std::string trial_id;
  int start_idx{-1};
  int end_idx{-1};
  nlohmann::json condition;          // arbitrary key/value metadata, written to CSV
  double expected_distance_m{0.0};   // 0 = skip distance/speed sanity check
  double expected_speed_mps{0.0};    // 0 = skip speed sanity check
};


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

  /* ---------------- Lifecycle (legacy + snapshot) ---------------- */

  void start()
  {
    resetAccumulators();
    active_ = false;  // gated by mission progress in legacy mode
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

  /* ---------------- Snapshot mode ---------------- */

  // Load per-leg metadata. After this returns true, the logger is in snapshot
  // mode: onProgressTick(idx) drives leg start/end transitions and one CSV
  // row is emitted per leg. Sets `error_msg` and returns false on parse error.
  //
  // Expected JSON shape:
  //   {
  //     "experiment": "straight_line",
  //     "csv_path": "/abs/path/to/output.csv",
  //     "speed_tolerance_mps": 0.3,
  //     "legs": [
  //       {
  //         "trial_id": "v04_t00",
  //         "start_idx": 3,
  //         "end_idx": 5,
  //         "condition": {"speed_mps": 4.0},
  //         "expected_distance_m": 50.0,
  //         "expected_speed_mps": 4.0
  //       }, ...
  //     ]
  //   }
  bool loadLegMetadata(const std::string & meta_path, std::string & error_msg)
  {
    clearLegMetadata();

    std::ifstream f(meta_path);
    if (!f.is_open()) {
      error_msg = "Failed to open per-leg metadata: " + meta_path;
      return false;
    }

    nlohmann::json meta;
    try {
      meta = nlohmann::json::parse(f);
    } catch (const std::exception & e) {
      error_msg = std::string("Per-leg metadata JSON parse error: ") + e.what();
      return false;
    }

    if (!meta.contains("legs") || !meta["legs"].is_array() ||
        meta["legs"].empty())
    {
      error_msg = "Per-leg metadata missing non-empty 'legs' array";
      return false;
    }
    if (!meta.contains("csv_path") || !meta["csv_path"].is_string()) {
      error_msg = "Per-leg metadata missing 'csv_path' string";
      return false;
    }

    experiment_ = meta.value("experiment", std::string{"unknown"});
    csv_path_ = meta["csv_path"].get<std::string>();
    speed_tolerance_mps_ = meta.value("speed_tolerance_mps", 0.3);

    for (const auto & item : meta["legs"]) {
      LegSpec leg;
      leg.trial_id = item.value("trial_id", std::string{});
      if (!item.contains("start_idx") || !item.contains("end_idx")) {
        error_msg = "Leg missing start_idx or end_idx";
        clearLegMetadata();
        return false;
      }
      leg.start_idx = item["start_idx"].get<int>();
      leg.end_idx = item["end_idx"].get<int>();
      if (item.contains("condition")) {
        leg.condition = item["condition"];
      }
      leg.expected_distance_m = item.value("expected_distance_m", 0.0);
      leg.expected_speed_mps = item.value("expected_speed_mps", 0.0);
      legs_.push_back(std::move(leg));
    }

    snapshot_mode_ = true;
    active_leg_idx_ = -1;
    return true;
  }

  void clearLegMetadata()
  {
    snapshot_mode_ = false;
    legs_.clear();
    active_leg_idx_ = -1;
    csv_path_.clear();
    experiment_.clear();
    speed_tolerance_mps_ = 0.3;
  }

  bool snapshotMode() const { return snapshot_mode_; }

  // For tests: inject synthetic samples as if from the /fmu subscriptions.
  // Production code must not call these.
  void injectBatterySampleForTest(
    uint64_t timestamp_us, double voltage_v, double current_a)
  {
    px4_msgs::msg::BatteryStatus msg;
    msg.timestamp = timestamp_us;
    msg.voltage_v = voltage_v;
    msg.current_a = current_a;
    batteryCallback(msg);
  }
  void injectPositionSampleForTest(double x, double y, bool xy_valid = true)
  {
    px4_msgs::msg::VehicleLocalPosition msg;
    msg.x = x;
    msg.y = y;
    msg.xy_valid = xy_valid;
    positionCallback(msg);
  }

  // Drive snapshot transitions. Call from Patrol's onProgressUpdate callback
  // for every reported `current_index`. No-op in legacy mode.
  //
  // Order: process leg-end matches first, then leg-start matches, so that an
  // index that is simultaneously end-of-leg-N and start-of-leg-(N+1) is
  // handled cleanly (row emitted, then accumulators reset for next leg).
  void onProgressTick(int current_index)
  {
    if (!snapshot_mode_) return;

    // 1. End-of-leg
    if (active_leg_idx_ >= 0 &&
        legs_[active_leg_idx_].end_idx == current_index)
    {
      emitLegRow(legs_[active_leg_idx_]);
      active_ = false;
      active_leg_idx_ = -1;
    }

    // 2. Start-of-leg
    for (size_t i = 0; i < legs_.size(); ++i) {
      if (legs_[i].start_idx == current_index) {
        resetAccumulators();
        start_time_ = node_.now();
        active_ = true;
        active_leg_idx_ = static_cast<int>(i);
        break;
      }
    }
  }

private:
  /* ---------------- Battery integration ---------------- */

  void batteryCallback(const px4_msgs::msg::BatteryStatus & msg)
  {
    if (!active_) return;

    if (!have_last_timestamp_) {
      last_timestamp_us_ = msg.timestamp;
      have_last_timestamp_ = true;
      return;
    }

    double dt = (msg.timestamp - last_timestamp_us_) * 1e-6; // us → s
    last_timestamp_us_ = msg.timestamp;

    double current = std::abs(msg.current_a);
    double power_w = msg.voltage_v * current;

    energy_j_ += power_w * dt;
    voltage_integral_v_s_ += msg.voltage_v * dt;
    integration_seconds_ += dt;
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

  /* ---------------- Helpers ---------------- */

  void resetAccumulators()
  {
    energy_j_ = 0.0;
    distance_m_ = 0.0;
    voltage_integral_v_s_ = 0.0;
    integration_seconds_ = 0.0;
    have_last_timestamp_ = false;
    last_timestamp_us_ = 0;
    have_last_pos_ = false;
  }

  void emitLegRow(const LegSpec & leg)
  {
    end_time_ = node_.now();
    const double duration_s = (end_time_ - start_time_).seconds();
    const double mean_power_w =
      (duration_s > 1e-6) ? (energy_j_ / duration_s) : 0.0;
    const double mean_speed_mps =
      (duration_s > 1e-6) ? (distance_m_ / duration_s) : 0.0;
    const double mean_voltage_v =
      (integration_seconds_ > 1e-6)
        ? (voltage_integral_v_s_ / integration_seconds_)
        : 0.0;

    bool passed_speed_check = true;
    if (leg.expected_speed_mps > 1e-6) {
      passed_speed_check =
        std::abs(mean_speed_mps - leg.expected_speed_mps) <=
        speed_tolerance_mps_;
    }

    const double utc_s =
      std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    const bool write_header = !std::filesystem::exists(csv_path_);
    std::ofstream out(csv_path_, std::ios::app);
    if (!out.is_open()) {
      RCLCPP_ERROR(node_.get_logger(),
        "EnergyLogger: failed to open per-leg CSV %s", csv_path_.c_str());
      return;
    }
    if (write_header) {
      out << "experiment,trial_id,condition_json,start_idx,end_idx,"
             "energy_j,distance_m,duration_s,"
             "mean_power_w,mean_speed_mps,mean_voltage_v,"
             "expected_distance_m,expected_speed_mps,"
             "passed_speed_check,utc_s\n";
    }
    // condition_json is JSON, possibly containing commas — wrap in double
    // quotes and escape internal quotes.
    std::string condition_str = leg.condition.dump();
    std::string condition_csv;
    condition_csv.reserve(condition_str.size() + 4);
    condition_csv.push_back('"');
    for (char c : condition_str) {
      if (c == '"') {
        condition_csv += "\"\"";  // CSV-escape
      } else {
        condition_csv.push_back(c);
      }
    }
    condition_csv.push_back('"');

    out << experiment_ << ","
        << leg.trial_id << ","
        << condition_csv << ","
        << leg.start_idx << ","
        << leg.end_idx << ","
        << energy_j_ << ","
        << distance_m_ << ","
        << duration_s << ","
        << mean_power_w << ","
        << mean_speed_mps << ","
        << mean_voltage_v << ","
        << leg.expected_distance_m << ","
        << leg.expected_speed_mps << ","
        << (passed_speed_check ? "true" : "false") << ","
        << utc_s << "\n";

    RCLCPP_INFO(node_.get_logger(),
      "EnergyLogger leg %s: E=%.1f J, d=%.2f m, t=%.2f s, P_mean=%.1f W, "
      "v_mean=%.2f m/s, V_mean=%.2f V, speed_check=%s",
      leg.trial_id.c_str(), energy_j_, distance_m_, duration_s, mean_power_w,
      mean_speed_mps, mean_voltage_v, passed_speed_check ? "PASS" : "FAIL");
  }

  /* ---------------- ROS ---------------- */

  rclcpp::Node & node_;

  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;

  /* ---------------- State ---------------- */

  bool active_{false};

  bool have_last_timestamp_{false};
  uint64_t last_timestamp_us_{0};
  double energy_j_{0.0};
  double voltage_integral_v_s_{0.0};
  double integration_seconds_{0.0};

  bool have_last_pos_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  double distance_m_{0.0};

  rclcpp::Time start_time_;
  rclcpp::Time end_time_;

  /* ---------------- Snapshot mode ---------------- */

  bool snapshot_mode_{false};
  std::string csv_path_;
  std::string experiment_;
  double speed_tolerance_mps_{0.3};
  std::vector<LegSpec> legs_;
  int active_leg_idx_{-1};
};
