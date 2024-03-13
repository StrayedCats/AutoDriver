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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include "auto_driver_msgs/action/get_angles_from_tf.hpp"

namespace auto_driver_interface
{

class TfToPositionActionServer : public rclcpp::Node
{
public:
  using TfToPosition = auto_driver_msgs::action::GetAnglesFromTf;
  using GoalHandleTfToPosition = rclcpp_action::ServerGoalHandle<TfToPosition>;

  explicit TfToPositionActionServer(const rclcpp::NodeOptions &);

private:
  rclcpp_action::Server<TfToPosition>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const TfToPosition::Goal>);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTfToPosition>);

  void handle_accepted(const std::shared_ptr<GoalHandleTfToPosition>);
  void execute(const std::shared_ptr<GoalHandleTfToPosition>);

  double z_offset_;
  int32_t retry_count_;
};

}  // namespace auto_driver_interface
