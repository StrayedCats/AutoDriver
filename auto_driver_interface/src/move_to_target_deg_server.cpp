// Copyright 2024 StrayedCats.
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

#include "auto_driver_interface/move_to_target_deg_server.hpp"

namespace auto_driver_interface
{

  MoveToTargetDegServer::MoveToTargetDegServer(const rclcpp::NodeOptions & options)
    : Node("move_to_target_deg_server", options)
  {
    this->yaw_publisher_ = this->create_publisher<std_msgs::msg::Int64>("can_node/gm6020_1/target_deg", 1);
    this->pitch_publisher_ = this->create_publisher<std_msgs::msg::Int64>("can_node/gm6020_0/target_deg", 1);

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<MoveToTargetDeg>(
      this,
      "move_to_target_deg",
      std::bind(&MoveToTargetDegServer::handle_goal, this, _1, _2),
      std::bind(&MoveToTargetDegServer::handle_cancel, this, _1),
      std::bind(&MoveToTargetDegServer::handle_accepted, this, _1));
  }

  rclcpp_action::GoalResponse MoveToTargetDegServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToTargetDeg::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveToTargetDegServer::handle_cancel(
    const std::shared_ptr<GoalHandleMoveToTargetDeg> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveToTargetDegServer::handle_accepted(const std::shared_ptr<GoalHandleMoveToTargetDeg> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal has been accepted");
    using namespace std::placeholders;
    std::thread{std::bind(&MoveToTargetDegServer::execute, this, _1), goal_handle}.detach();
  }

  void MoveToTargetDegServer::execute(const std::shared_ptr<GoalHandleMoveToTargetDeg> goal_handle)
  {
    auto result = std::make_shared<MoveToTargetDeg::Result>();
    auto goal = goal_handle->get_goal();
    auto msec = goal->msec;

    auto yaw_msg = std_msgs::msg::Int64();
    auto pitch_msg = std_msgs::msg::Int64();
    yaw_msg.data = goal->yaw_deg + (goal->yaw_deg + center_offset_yaw_) * speed_multiplier_yaw_;
    pitch_msg.data = goal->pitch_deg + (goal->pitch_deg + center_offset_pitch_) * speed_multiplier_pitch_;
    this->yaw_publisher_->publish(yaw_msg);
    this->pitch_publisher_->publish(pitch_msg);

    RCLCPP_INFO(this->get_logger(), "Publishing target degrees: yaw: %d, pitch: %d", yaw_msg.data, pitch_msg.data);

    rclcpp::sleep_for(std::chrono::milliseconds(msec));

    result->succeed = true;
    goal_handle->succeed(result);
  }
}  // namespace auto_driver_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(auto_driver_interface::MoveToTargetDegServer)