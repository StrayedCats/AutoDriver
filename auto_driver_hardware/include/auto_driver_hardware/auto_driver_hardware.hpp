#pragma once

#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace auto_driver_hardware
{
using hardware_interface::CallbackReturn;
using rclcpp_lifecycle::State;
using hardware_interface::return_type;

class AutoDriverHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  CallbackReturn on_activate(const State &) override;
  CallbackReturn on_deactivate(const State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> position_commands_;
  std::vector<double> velocity_commands_;
  double shoot_command_;
  double shoot_state_;
};
}  // namespace auto_driver_hardware
