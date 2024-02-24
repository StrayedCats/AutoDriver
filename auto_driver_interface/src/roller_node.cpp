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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace auto_driver_interface
{

class RollerNode : public rclcpp::Node
{

public:
  RollerNode(rclcpp::NodeOptions options)
  : Node("node", options)
  {
    this->declare_parameter<bool>("invert", false);
    invert = this->get_parameter("invert").as_bool();

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_current", 1);
    target_current_sub = this->create_subscription<std_msgs::msg::Float64>(
      "/target_current", 1, std::bind(&RollerNode::target_current_callback, this, std::placeholders::_1));

    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&RollerNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float64();
    message.data = target_current * (invert ? -1 : 1);
    publisher_->publish(message);
  }

  void target_current_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    target_current = msg->data;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_current_sub;
  rclcpp::TimerBase::SharedPtr timer_;

  float target_current = 0.0;
  bool invert = false;
};

}  // namespace auto_driver_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(auto_driver_interface::RollerNode)
