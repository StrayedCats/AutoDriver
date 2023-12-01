#include "auto_driver_hardware/auto_driver_hardware.hpp"

namespace auto_driver_hardware
{
AutoDriverHardware::CallbackReturn AutoDriverHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // TODO : Initialize AutoDriverInterface

  this->position_states_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  this->velocity_states_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  this->position_commands_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  this->velocity_commands_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  this->shoot_command_ = 0;

  return CallbackReturn::SUCCESS;
}

CallbackReturn AutoDriverHardware::on_activate(const State & state)
{
  (void)state;
  for (size_t i = 0; i < this->info_.joints.size(); ++i) {
    if (std::isnan(this->position_states_.at(i))) {
      this->position_states_.at(i) = 0.0;
    }
    if (std::isnan(this->velocity_states_.at(i))) {
      this->velocity_states_.at(i) = 0.0;
    }
    if (std::isnan(this->velocity_commands_.at(i))) {
      this->velocity_commands_.at(i) = 0.0;
    }
  }

  this->shoot_command_ = 0;

  // TODO : on_activate AutoDriverInterface
  return CallbackReturn::SUCCESS;
}

CallbackReturn AutoDriverHardware::on_deactivate(const State & state)
{
  (void)state;
  // TODO : on_deactivate AutoDriverInterface
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AutoDriverHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // joint states
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->info_.joints.at(i).name, hardware_interface::HW_IF_POSITION,
        &position_states_.at(i)));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
        &velocity_states_.at(i)));
  }

  // shoot state
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      this->info_.gpios[0].name, "shoot_state", &this->shoot_state_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AutoDriverHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // joint commands
  for (size_t i = 0; i < this->info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
        &this->velocity_commands_.at(i)));
  }

  // shoot command
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      this->info_.gpios[0].name, "shoot_command", &this->shoot_command_));

  return command_interfaces;
}

return_type AutoDriverHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{

  const double axis_velocity_0 = 0.0;
  const double axis_velocity_1 = 0.0;
  // TODO : read axis_0_velocity and axis_1_velocity from AutoDriverInterface

  if (!std::isnan(axis_velocity_0)) {
    // TODO : set this->velocity_states_[0]
  }
  if (!std::isnan(axis_velocity_1)) {
    // TODO : set this->velocity_states_[1]
  }

  const double axis_0_position = 0.0;
  const double axis_1_position = 0.0;
  // TODO : read axis_0 and axis_1 from AutoDriverInterface

  if (!std::isnan(axis_0_position)) {
    // TODO : set this->position_states_[0]
  }
  if (!std::isnan(axis_1_position)) {
    // TODO : set this->position_states_[1]
  }

  // TODO : cleanup
  return return_type::OK;
}

return_type AutoDriverHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO : use this->velocity_commands_.at(0) and this->velocity_commands_.at(1)

  // TODO : write this->shoot_command_ for shooting

  return return_type::OK;
}
} // namespace auto_driver_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  auto_driver_hardware::AutoDriverHardware,
  hardware_interface::SystemInterface)
