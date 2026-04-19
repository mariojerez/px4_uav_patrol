/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 *               2026 Mario Jerez (action-server wrapper)
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <shepherding_msgs/action/execute_patrol.hpp>

#include <patrol.hpp>

using namespace std::chrono_literals;
using ExecutePatrol = shepherding_msgs::action::ExecutePatrol;
using GoalHandleExecutePatrol = rclcpp_action::ServerGoalHandle<ExecutePatrol>;

static const char * kNodeName = "patrol_mission";
static const char * kActionName = "execute_patrol";   // resolves to ./execute_patrol under the node namespace
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
    if (goal->mission_file.empty()) {
      RCLCPP_WARN(node_->get_logger(),
        "Rejecting patrol goal: empty mission_file");
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
    std::lock_guard<std::mutex> lock(mutex_);
    active_goal_handle_ = goal_handle;

    const auto goal = goal_handle->get_goal();
    std::string error;
    if (!patrol_.start(goal->mission_file, error)) {
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
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  std::mutex mutex_;
  std::shared_ptr<GoalHandleExecutePatrol> active_goal_handle_;
  double feedback_rate_hz_{2.0};
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
