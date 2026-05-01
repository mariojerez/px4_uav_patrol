/****************************************************************************
 * Copyright (c) 2026 Mario Jerez
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "energy_logger.hpp"

namespace fs = std::filesystem;

namespace {

// Write a meta JSON to disk and return its path. The CSV path is also
// returned via out_csv_path so the test can inspect / delete the output.
fs::path writeMetaJson(
  const std::string & body, fs::path & out_csv_path,
  const std::string & test_name)
{
  fs::path tmp_dir = fs::temp_directory_path() / ("ut_" + test_name);
  fs::create_directories(tmp_dir);
  fs::path meta_path = tmp_dir / "meta.json";
  out_csv_path = tmp_dir / "snapshot.csv";

  // Replace the placeholder so the meta JSON points at the per-test CSV.
  std::string with_csv = body;
  const std::string placeholder = "@@CSV@@";
  size_t pos = with_csv.find(placeholder);
  if (pos != std::string::npos) {
    with_csv.replace(pos, placeholder.size(), out_csv_path.string());
  }

  std::ofstream(meta_path) << with_csv;

  // Make sure no leftover CSV from a previous run leaks into the test.
  if (fs::exists(out_csv_path)) {
    fs::remove(out_csv_path);
  }
  return meta_path;
}

// Read the data rows (excluding header) from the CSV and split by comma.
std::vector<std::vector<std::string>> readCsvRows(const fs::path & path)
{
  std::vector<std::vector<std::string>> rows;
  std::ifstream in(path);
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    if (first) {
      first = false;
      continue;
    }
    std::vector<std::string> fields;
    // Simple CSV split — does not handle the quoted condition_json field's
    // internal commas. We strip the quoted field separately.
    bool in_quotes = false;
    std::string current;
    for (char c : line) {
      if (c == '"') {
        in_quotes = !in_quotes;
        current.push_back(c);
      } else if (c == ',' && !in_quotes) {
        fields.push_back(current);
        current.clear();
      } else {
        current.push_back(c);
      }
    }
    if (!current.empty()) fields.push_back(current);
    rows.push_back(std::move(fields));
  }
  return rows;
}

class RclcppEnv : public ::testing::Environment
{
public:
  void SetUp() override
  {
    // The repo's $CYCLONEDDS_URI pins the RMW to wlan0 for in-the-field
    // multi-drone use; that interface doesn't exist in CI / dev containers,
    // and create_subscription() inside the EnergyLogger constructor would
    // throw "rmw handle is invalid". Unset it so Cyclone falls back to its
    // default (auto-pick available interface, including lo).
    unsetenv("CYCLONEDDS_URI");
    rclcpp::init(0, nullptr);
  }
  void TearDown() override { rclcpp::shutdown(); }
};

}  // namespace

TEST(EnergyLoggerSnapshot, LegacyModeUnchanged)
{
  auto node = rclcpp::Node::make_shared("test_logger_legacy");
  EnergyLogger logger(*node);
  logger.start();
  EXPECT_FALSE(logger.snapshotMode());

  logger.enable();
  // 1 second of 240 W constant (16 V * 15 A) with positions tracing 5 m.
  logger.injectBatterySampleForTest(0,         16.0, 15.0);
  logger.injectPositionSampleForTest(0.0, 0.0);
  logger.injectBatterySampleForTest(1'000'000, 16.0, 15.0);
  logger.injectPositionSampleForTest(5.0, 0.0);

  EXPECT_NEAR(logger.energyJoules(), 240.0, 1e-3);
  EXPECT_NEAR(logger.distanceMeters(), 5.0, 1e-3);

  logger.disable();
  // Samples while disabled must not accumulate.
  logger.injectBatterySampleForTest(2'000'000, 16.0, 15.0);
  logger.injectPositionSampleForTest(50.0, 0.0);
  EXPECT_NEAR(logger.energyJoules(), 240.0, 1e-3);
  EXPECT_NEAR(logger.distanceMeters(), 5.0, 1e-3);
}

TEST(EnergyLoggerSnapshot, EmitsRowOnLegEnd)
{
  auto node = rclcpp::Node::make_shared("test_logger_one_leg");
  EnergyLogger logger(*node);
  logger.start();

  fs::path csv_path;
  fs::path meta = writeMetaJson(R"({
    "experiment": "straight_line",
    "csv_path": "@@CSV@@",
    "speed_tolerance_mps": 0.5,
    "legs": [
      {
        "trial_id": "v04_t00",
        "start_idx": 1,
        "end_idx": 3,
        "condition": {"speed_mps": 4.0},
        "expected_distance_m": 50.0,
        "expected_speed_mps": 4.0
      }
    ]
  })", csv_path, "leg_end");

  std::string err;
  ASSERT_TRUE(logger.loadLegMetadata(meta.string(), err)) << err;
  EXPECT_TRUE(logger.snapshotMode());

  // Index 0 is irrelevant to legs; nothing should happen.
  logger.onProgressTick(0);
  ASSERT_FALSE(fs::exists(csv_path));

  // Start the leg.
  logger.onProgressTick(1);
  // Sleep so duration_s is positive (the logger uses node_.now()).
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Inject 100 samples over 12.5 s of simulated battery time at 240 W,
  // distance walks from 0 to 50 m.
  for (int i = 0; i <= 100; ++i) {
    uint64_t ts_us = static_cast<uint64_t>(i) * 125'000;  // 0.125 s steps
    logger.injectBatterySampleForTest(ts_us, 16.0, 15.0);
    logger.injectPositionSampleForTest(0.5 * i, 0.0);
  }

  // End the leg → CSV row appears.
  logger.onProgressTick(3);
  ASSERT_TRUE(fs::exists(csv_path));

  auto rows = readCsvRows(csv_path);
  ASSERT_EQ(rows.size(), 1u);

  // Columns:
  //   experiment,trial_id,condition_json,start_idx,end_idx,
  //   energy_j,distance_m,duration_s,
  //   mean_power_w,mean_speed_mps,mean_voltage_v,
  //   expected_distance_m,expected_speed_mps,
  //   passed_speed_check,utc_s
  EXPECT_EQ(rows[0][0], "straight_line");
  EXPECT_EQ(rows[0][1], "v04_t00");
  EXPECT_EQ(rows[0][3], "1");
  EXPECT_EQ(rows[0][4], "3");
  // 100 dt-intervals of 0.125 s = 12.5 s of integrated battery time at 240 W.
  EXPECT_NEAR(std::stod(rows[0][5]), 12.5 * 240.0, 1e-3);
  EXPECT_NEAR(std::stod(rows[0][6]), 50.0, 1e-3);
  // distance/duration is wall-clock-dependent, so just check it's set; actual
  // duration_s field exists at index 7.
  EXPECT_GT(std::stod(rows[0][7]), 0.0);
  // Mean voltage = 16.0 V (constant in the test).
  EXPECT_NEAR(std::stod(rows[0][10]), 16.0, 1e-3);
  EXPECT_EQ(rows[0][11], "50");
  EXPECT_EQ(rows[0][12], "4");
}

TEST(EnergyLoggerSnapshot, BackToBackLegsTransitionCleanly)
{
  auto node = rclcpp::Node::make_shared("test_logger_back_to_back");
  EnergyLogger logger(*node);
  logger.start();

  fs::path csv_path;
  fs::path meta = writeMetaJson(R"({
    "experiment": "hover",
    "csv_path": "@@CSV@@",
    "speed_tolerance_mps": 0.5,
    "legs": [
      {"trial_id": "h0", "start_idx": 2, "end_idx": 4, "condition": {}},
      {"trial_id": "h1", "start_idx": 4, "end_idx": 6, "condition": {}}
    ]
  })", csv_path, "back_to_back");

  std::string err;
  ASSERT_TRUE(logger.loadLegMetadata(meta.string(), err)) << err;

  // Leg h0
  logger.onProgressTick(2);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  logger.injectBatterySampleForTest(0,         16.0, 15.0);
  logger.injectBatterySampleForTest(1'000'000, 16.0, 15.0);
  logger.injectPositionSampleForTest(0.0, 0.0);
  logger.injectPositionSampleForTest(0.0, 0.0);

  // idx 4 = end-of-h0 AND start-of-h1
  logger.onProgressTick(4);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Leg h1 — different power level (16 V * 20 A = 320 W) for 1 s.
  logger.injectBatterySampleForTest(0,         16.0, 20.0);
  logger.injectBatterySampleForTest(1'000'000, 16.0, 20.0);
  logger.injectPositionSampleForTest(0.0, 0.0);
  logger.injectPositionSampleForTest(0.0, 0.0);

  logger.onProgressTick(6);

  auto rows = readCsvRows(csv_path);
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0][1], "h0");
  EXPECT_EQ(rows[1][1], "h1");
  // h0 = 240 J (240 W * 1 s); h1 = 320 J (320 W * 1 s) — accumulators were
  // reset at the back-to-back transition, so h1 must NOT include h0's energy.
  EXPECT_NEAR(std::stod(rows[0][5]), 240.0, 1e-3);
  EXPECT_NEAR(std::stod(rows[1][5]), 320.0, 1e-3);
}

TEST(EnergyLoggerSnapshot, SpeedToleranceFlagsSlowTrials)
{
  auto node = rclcpp::Node::make_shared("test_logger_speed_check");
  EnergyLogger logger(*node);
  logger.start();

  fs::path csv_path;
  fs::path meta = writeMetaJson(R"({
    "experiment": "straight_line",
    "csv_path": "@@CSV@@",
    "speed_tolerance_mps": 0.3,
    "legs": [
      {"trial_id": "fast", "start_idx": 1, "end_idx": 2, "condition": {},
       "expected_distance_m": 50.0, "expected_speed_mps": 4.0},
      {"trial_id": "slow", "start_idx": 3, "end_idx": 4, "condition": {},
       "expected_distance_m": 50.0, "expected_speed_mps": 4.0}
    ]
  })", csv_path, "speed_check");

  std::string err;
  ASSERT_TRUE(logger.loadLegMetadata(meta.string(), err)) << err;

  // FAST leg: 5 m in 1 s wall-clock-ish — speed_mps ~ 5 m/s, expected 4 m/s,
  // tolerance 0.3 → outside tolerance → FAIL. (Actual mean_speed depends on
  // node_.now() but the field reports distance/duration.)
  logger.onProgressTick(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  logger.injectBatterySampleForTest(0, 16.0, 15.0);
  logger.injectBatterySampleForTest(50'000, 16.0, 15.0);  // 0.05 s
  logger.injectPositionSampleForTest(0.0, 0.0);
  logger.injectPositionSampleForTest(50.0 * 0.05, 0.0);  // ~2.5 m
  logger.onProgressTick(2);

  // SLOW leg: distance 0 → speed 0 → fails check.
  logger.onProgressTick(3);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  logger.injectBatterySampleForTest(0, 16.0, 15.0);
  logger.injectBatterySampleForTest(50'000, 16.0, 15.0);
  logger.injectPositionSampleForTest(0.0, 0.0);
  logger.injectPositionSampleForTest(0.0, 0.0);
  logger.onProgressTick(4);

  auto rows = readCsvRows(csv_path);
  ASSERT_EQ(rows.size(), 2u);
  // passed_speed_check is column index 13 (last data column before utc_s).
  // Both should fail (one too fast, one zero distance).
  EXPECT_EQ(rows[0][13], "false");
  EXPECT_EQ(rows[1][13], "false");
}

TEST(EnergyLoggerSnapshot, RejectsMalformedMeta)
{
  auto node = rclcpp::Node::make_shared("test_logger_bad_meta");
  EnergyLogger logger(*node);
  std::string err;

  fs::path csv_path;
  fs::path missing_legs = writeMetaJson(R"({"csv_path": "@@CSV@@"})", csv_path,
    "bad_meta_no_legs");
  EXPECT_FALSE(logger.loadLegMetadata(missing_legs.string(), err));
  EXPECT_FALSE(logger.snapshotMode());

  fs::path missing_csv = writeMetaJson(
    R"({"legs": [{"trial_id": "a", "start_idx": 1, "end_idx": 2}]})",
    csv_path, "bad_meta_no_csv");
  EXPECT_FALSE(logger.loadLegMetadata(missing_csv.string(), err));

  fs::path missing_idx = writeMetaJson(
    R"({"csv_path": "@@CSV@@", "legs": [{"trial_id": "a"}]})",
    csv_path, "bad_meta_no_idx");
  EXPECT_FALSE(logger.loadLegMetadata(missing_idx.string(), err));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new RclcppEnv);
  return RUN_ALL_TESTS();
}
