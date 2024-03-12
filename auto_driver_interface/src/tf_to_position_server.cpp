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

#include "auto_driver_interface/tf_to_position_server.hpp"

namespace auto_driver_interface
{

  TfToPositionActionServer::TfToPositionActionServer(const rclcpp::NodeOptions & options)
    : Node("tf_to_position_action_server", options)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<TfToPosition>(
      this,
      "tf_to_position",
      std::bind(&TfToPositionActionServer::handle_goal, this, _1, _2),
      std::bind(&TfToPositionActionServer::handle_cancel, this, _1),
      std::bind(&TfToPositionActionServer::handle_accepted, this, _1));
  }

  rclcpp_action::GoalResponse TfToPositionActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TfToPosition::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    
    if (goal->root_tf_frame_id.empty() || goal->camera_tf_frame_id.empty() || goal->target_tf_frame_id.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid goal request");
      return rclcpp_action::GoalResponse::REJECT;
    }


    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse TfToPositionActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleTfToPosition> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void TfToPositionActionServer::handle_accepted(const std::shared_ptr<GoalHandleTfToPosition> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal has been accepted");
    using namespace std::placeholders;
    std::thread{std::bind(&TfToPositionActionServer::execute, this, _1), goal_handle}.detach();
  }

  void TfToPositionActionServer::execute(const std::shared_ptr<GoalHandleTfToPosition> goal_handle)
  {
    auto result = std::make_shared<TfToPosition::Result>();

    // Calculate tf
    auto goal = goal_handle->get_goal();

    auto tf_root = goal->root_tf_frame_id;
    auto tf_from = goal->camera_tf_frame_id;
    auto tf_to = goal->target_tf_frame_id;
    auto retry_count = 5;
    geometry_msgs::msg::TransformStamped transform;

    for (int i = 0; i < retry_count; i++)
    {
      try {
        transform = tf_buffer_->lookupTransform(tf_root, tf_from, tf2::TimePointZero);
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        continue;
      }

      geometry_msgs::msg::TransformStamped transform2;
      try {
        transform2 = tf_buffer_->lookupTransform(tf_root, tf_to, tf2::TimePointZero);
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        continue;
      }
  
      auto x = transform2.transform.translation.x - transform.transform.translation.x;
      auto y = transform2.transform.translation.y - transform.transform.translation.y;
      auto z = transform2.transform.translation.z - transform.transform.translation.z;
      auto current_rotation = transform.transform.rotation;

      auto yaw = atan2(y, x);
      yaw = (x < 0) ? -yaw : yaw;
      auto degree = yaw * 180 / M_PI;

      auto x_y = sqrt(x * x + y * y);
      auto pitch = atan2(x_y, z);
      pitch = (z < 0) ? -pitch : pitch;
      pitch = pitch * 180 / M_PI;

      result->yaw_deg = static_cast<int32_t>(degree);
      result->pitch_deg = static_cast<int32_t>(pitch);
      result->succeed = true;

      goal_handle->succeed(result);
      return;
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to calculate tf");
    result->succeed = false;
    goal_handle->succeed(result);
  }
}  // namespace auto_driver_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(auto_driver_interface::TfToPositionActionServer)