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
#include <std_msgs/msg/empty.hpp>

namespace auto_driver_interface
{
class HammerNode : public rclcpp::Node
{
public:
  HammerNode(const rclcpp::NodeOptions & options)
  : Node("hammer", options), trigger(false)
  {
    subscription = this->create_subscription<std_msgs::msg::Empty>(
      "/hammer", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
        if (!trigger) {
          trigger = true;
          push();
          pull_timer = this->create_wall_timer(
            std::chrono::milliseconds(800), std::bind(&HammerNode::pull, this));
          clear_timer = this->create_wall_timer(
            std::chrono::milliseconds(1600), std::bind(&HammerNode::clear, this));
        }
      });

    servo0_pub = this->create_publisher<std_msgs::msg::Float64>("degree", 10);
  }

private:
  void hammer(double value)
  {
    auto message = std_msgs::msg::Float64();
    message.data = value;
    servo0_pub->publish(message);
  }

  void push()
  {
    RCLCPP_INFO(this->get_logger(), "push");
    hammer(85.0);
  }

  void pull()
  {
    hammer(0.0);
    RCLCPP_INFO(this->get_logger(), "pull");
    pull_timer->cancel();
  }

  void clear()
  {
    trigger = false;
    RCLCPP_INFO(this->get_logger(), "clear");
    clear_timer->cancel();
  }

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo0_pub;
  rclcpp::TimerBase::SharedPtr pull_timer;
  rclcpp::TimerBase::SharedPtr clear_timer;
  bool trigger;
};
} // namespace auto_driver_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(auto_driver_interface::HammerNode)
