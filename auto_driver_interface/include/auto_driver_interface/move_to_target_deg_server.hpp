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

#pragma once

#include <memory>
#include <thread>

#include <std_msgs/msg/int64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "auto_driver_msgs/action/move_to_target_deg.hpp"

namespace auto_driver_interface
{

class MoveToTargetDegServer : public rclcpp::Node
{
public:
  using MoveToTargetDeg = auto_driver_msgs::action::MoveToTargetDeg;
  using GoalHandleMoveToTargetDeg = rclcpp_action::ServerGoalHandle<MoveToTargetDeg>;

  explicit MoveToTargetDegServer(const rclcpp::NodeOptions &);

private:
  double center_offset_yaw_ = -180.0;
  double center_offset_pitch_ = -90.0;

  double speed_multiplier_yaw_ = 0.5;
  double speed_multiplier_pitch_ = 0.5;

  rclcpp_action::Server<MoveToTargetDeg>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToTargetDeg::Goal>);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToTargetDeg>);

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToTargetDeg>);

  void execute(const std::shared_ptr<GoalHandleMoveToTargetDeg>);

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr yaw_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pitch_publisher_;
};

}  // namespace auto_driver_interface
