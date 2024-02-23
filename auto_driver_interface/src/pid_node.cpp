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
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/bool.hpp>

#include "pid.hpp"

namespace auto_driver_interface
{

class PidNode : public rclcpp::Node
{

public:
  PidNode(rclcpp::NodeOptions options)
  : Node("node", options)
  {
    this->declare_parameter<int>("init_deg", 180);
    this->declare_parameter<int>("p", 100);
    this->declare_parameter<int>("i", 60);
    this->declare_parameter<int>("d", 10);
    this->declare_parameter<int>("max_spd", 5000);
    this->declare_parameter<int>("min_limit", 25);
    this->declare_parameter<int>("max_limit", 315);

    this->get_parameter("init_deg", target_deg);
    this->get_parameter("p", p);
    this->get_parameter("i", i);
    this->get_parameter("d", d);
    this->get_parameter("max_spd", max_spd);
    this->get_parameter("min_limit", min_limit);
    this->get_parameter("max_limit", max_limit);

    publisher_ = this->create_publisher<std_msgs::msg::Int64>("target_volt", 1);

    current_deg_sub = this->create_subscription<std_msgs::msg::Float64>(
      "degree", 1, [&](const std_msgs::msg::Float64::SharedPtr msg) {
        current_deg = msg->data;
      });
    target_deg_sub = this->create_subscription<std_msgs::msg::Int64>(
      "target_deg", 1, [&](const std_msgs::msg::Int64::SharedPtr msg) {
        if (msg->data > max_limit) {
          RCLCPP_WARN(this->get_logger(), "Target degree is over max limit");
          target_deg = max_limit;
        } else if (msg->data < min_limit) {
          RCLCPP_WARN(this->get_logger(), "Target degree is under min limit");
          target_deg = min_limit;
        } else {
          target_deg = msg->data;
        }
      });

    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&PidNode::timer_callback, this));
    pid = std::make_shared<PID>(p, i, d, max_spd, -max_spd, 0.005);

  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int64();
    message.data = pid->calculate(target_deg, current_deg);
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pitch_publisher_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr minus_stop_trigger_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr plus_stop_trigger_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pitch_down_stop_trigger_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pitch_up_stop_trigger_sub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr target_deg_sub;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_deg_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_current_deg_sub;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<PID> pid;

  float current_deg = 180.0;

  int target_deg = 180.0;
  float pitch_target_deg = 90.0;

  int p = 50;
  int i = 3;
  int d = 0;
  int max_spd = 5000;
  int min_limit = 25;
  int max_limit = 315;
};

}  // namespace auto_driver_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(auto_driver_interface::PidNode)
