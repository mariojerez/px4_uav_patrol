/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 *               2026 Mario Jerez (action-server wrapper)
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <shepherding_msgs/action/execute_patrol.hpp>
#include <shepherding_msgs/srv/generate_patrol_mission.hpp>

#include <patrol.hpp>

using namespace std::chrono_literals;
using ExecutePatrol = shepherding_msgs::action::ExecutePatrol;
using GoalHandleExecutePatrol = rclcpp_action::ServerGoalHandle<ExecutePatrol>;

static const char * kNodeName = "patrol_mission";
static const char * kActionName = "execute_patrol";   // resolves to /execute_patrol at the node's namespace (default "/")
static const bool kEnableDebugOutput = true;

class PatrolActionServer
{
public:
  explicit PatrolActionServer(const std::shared_ptr<rclcpp::Node> & node)
  : node_(node),
    patrol_(node)
  {
    feedback_rate_hz_ =
      node_->declare_parameter<double>("patrol.feedback_rate_hz", 2.0);
    planner_service_timeout_s_ =
      node_->declare_parameter<double>(
        "patrol.planner_service_timeout_s", 600.0);

    action_server_ = rclcpp_action::create_server<ExecutePatrol>(
      node_,
      kActionName,
      std::bind(&PatrolActionServer::handleGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&PatrolActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&PatrolActionServer::handleAccepted, this, std::placeholders::_1));

    const auto period =
      std::chrono::duration<double>(1.0 / std::max(0.1, feedback_rate_hz_));
    feedback_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PatrolActionServer::tick, this));

    RCLCPP_INFO(node_->get_logger(),
      "Patrol action server ready on '%s' (feedback %.1f Hz)",
      kActionName, feedback_rate_hz_);
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const ExecutePatrol::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_handle_) {
      RCLCPP_WARN(node_->get_logger(),
        "Rejecting new patrol goal: a mission is already active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->mission_file.empty() && goal->farm_config.empty()) {
      RCLCPP_WARN(node_->get_logger(),
        "Rejecting patrol goal: both mission_file and farm_config are empty");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(node_->get_logger(),
      "Accepting patrol goal for mission: %s",
      goal->mission_file.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleExecutePatrol> /*goal_handle*/)
  {
    RCLCPP_INFO(node_->get_logger(), "Patrol cancel requested");
    patrol_.cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleExecutePatrol> goal_handle)
  {
    // Action callbacks run on the rclcpp executor thread. patrol_.start()
    // blocks for up to 15 s inside px4_ros2::waitForFMU, which would freeze
    // the single-threaded executor and prevent the freshly-created
    // VehicleStatus subscription from ever being matched / delivered.
    // Move the blocking startup to a worker thread so the executor keeps
    // spinning.
    std::thread([this, goal_handle]() {
      std::lock_guard<std::mutex> lock(mutex_);
      active_goal_handle_ = goal_handle;

      const auto goal = goal_handle->get_goal();
      std::string error;
      std::string mission_path = goal->mission_file;

      if (!goal->farm_config.empty()) {
        if (!resolveMissionFromPlanner(*goal, mission_path, error)) {
          RCLCPP_ERROR(node_->get_logger(),
            "Failed to resolve mission from planner: %s", error.c_str());
          auto result = std::make_shared<ExecutePatrol::Result>();
          result->success = false;
          result->energy_joules = 0.0f;
          result->distance_meters = 0.0f;
          goal_handle->abort(result);
          active_goal_handle_.reset();
          return;
        }
      }

      if (!patrol_.start(mission_path, error,
            goal->per_leg_meta_file, goal->auto_arm))
      {
        RCLCPP_ERROR(node_->get_logger(),
          "Failed to start patrol: %s", error.c_str());
        auto result = std::make_shared<ExecutePatrol::Result>();
        result->success = false;
        result->energy_joules = 0.0f;
        result->distance_meters = 0.0f;
        goal_handle->abort(result);
        active_goal_handle_.reset();
        return;
      }
    }).detach();
  }

  // Synchronously call the patrol planner to obtain a mission file path.
  // Returns true on success and writes the resulting on-disk path into
  // out_path. On failure writes a human-readable description into error_msg.
  bool resolveMissionFromPlanner(
    const ExecutePatrol::Goal & goal, std::string & out_path,
    std::string & error_msg)
  {
    using GeneratePatrolMission = shepherding_msgs::srv::GeneratePatrolMission;
    if (!planner_client_) {
      planner_client_ = node_->create_client<GeneratePatrolMission>(
        "/generate_patrol_mission");
    }
    if (!planner_client_->wait_for_service(std::chrono::seconds(2))) {
      error_msg = "patrol planner service /generate_patrol_mission unavailable";
      return false;
    }

    auto req = std::make_shared<GeneratePatrolMission::Request>();
    req->farm_config = goal.farm_config;
    req->cell_size_m = goal.cell_size_m;
    req->altitude_m = goal.altitude_m;
    req->groundspeed_mps = goal.groundspeed_mps;
    req->wind_east_mps = goal.wind_east_mps;
    req->wind_north_mps = goal.wind_north_mps;
    req->num_drones = goal.num_drones;
    req->drone_index = goal.drone_index;
    req->solver_time_limit_s = 0;
    req->force_regenerate = false;

    auto fut = planner_client_->async_send_request(req);
    const auto timeout = std::chrono::duration<double>(planner_service_timeout_s_);
    const auto timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
    if (fut.wait_for(timeout_ns) != std::future_status::ready) {
      error_msg = "patrol planner timed out (raise patrol.planner_service_timeout_s)";
      return false;
    }
    auto resp = fut.get();
    if (!resp->success) {
      error_msg = "patrol planner returned failure: " + resp->error_message;
      return false;
    }
    out_path = resp->mission_file;
    return true;
  }

  // Called at feedback_rate_hz_. Publishes feedback while the mission is
  // running; when the mission completes or is aborted, finalizes the goal.
  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_goal_handle_) {
      return;
    }

    auto feedback = std::make_shared<ExecutePatrol::Feedback>();
    feedback->progress_percent = patrol_.progressPercent();
    feedback->current_waypoint = patrol_.currentWaypoint();
    active_goal_handle_->publish_feedback(feedback);

    // Honor client-side cancellation requests.
    if (active_goal_handle_->is_canceling()) {
      patrol_.cancel();
      auto result = std::make_shared<ExecutePatrol::Result>();
      result->success = false;
      result->energy_joules = static_cast<float>(patrol_.energyJoules());
      result->distance_meters = static_cast<float>(patrol_.distanceMeters());
      active_goal_handle_->canceled(result);
      active_goal_handle_.reset();
      return;
    }

    if (patrol_.isComplete()) {
      auto result = std::make_shared<ExecutePatrol::Result>();
      result->success = true;
      result->energy_joules = static_cast<float>(patrol_.energyJoules());
      result->distance_meters = static_cast<float>(patrol_.distanceMeters());
      active_goal_handle_->succeed(result);
      active_goal_handle_.reset();
      return;
    }

    if (patrol_.wasAborted() && !patrol_.isActive()) {
      auto result = std::make_shared<ExecutePatrol::Result>();
      result->success = false;
      result->energy_joules = static_cast<float>(patrol_.energyJoules());
      result->distance_meters = static_cast<float>(patrol_.distanceMeters());
      if (active_goal_handle_->is_canceling()) {
        active_goal_handle_->canceled(result);
      } else {
        active_goal_handle_->abort(result);
      }
      active_goal_handle_.reset();
      return;
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  Patrol patrol_;

  rclcpp_action::Server<ExecutePatrol>::SharedPtr action_server_;
  rclcpp::Client<shepherding_msgs::srv::GeneratePatrolMission>::SharedPtr planner_client_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  std::mutex mutex_;
  std::shared_ptr<GoalHandleExecutePatrol> active_goal_handle_;
  double feedback_rate_hz_{2.0};
  double planner_service_timeout_s_{600.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(kNodeName);

  if (kEnableDebugOutput) {
    auto ret = rcutils_logging_set_logger_level(
      node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(node->get_logger(),
        "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }
  }

  PatrolActionServer server(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
