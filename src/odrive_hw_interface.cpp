
#include <limits>
#include <vector>

#include "odrive_hw_interface/odrive_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "odrive-can.hpp"

namespace odrive_hw_interface
{
hardware_interface::CallbackReturn OdriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::SUCCESS;
  }
   RCLCPP_INFO(
      rclcpp::get_logger("OdriveHardware"), "Interface init");

  hw_joint_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_joint_command_ = std::numeric_limits<double>::quiet_NaN();

  const hardware_interface::ComponentInfo & joint = info_.joints[0];

  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("OdriveHardware"),
      "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
      joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("OdriveHardware"),
      "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
      joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.state_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("OdriveHardware"), "Joint '%s' has %zu state interface. 1 expected.",
      joint.name.c_str(), joint.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("OdriveHardware"), "Joint '%s' have %s state interface. '%s' expected.",
      joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
      hardware_interface::HW_IF_EFFORT);
    return hardware_interface::CallbackReturn::ERROR;
  }

  int id = 0;
  char can_if[] = "can0";
  this->odrive =  new OdriveCAN(id,can_if);
  

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> OdriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_joint_state_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OdriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_joint_command_));
  return command_interfaces;
}

hardware_interface::CallbackReturn OdriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  // TODO(anyone): prepare the robot to receive commands
  this->odrive->motor_on();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands
  this->odrive->motor_off();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OdriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states
  this->odrive->update_motor_state();
  hw_joint_state_  = this->odrive->get_pos();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OdriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  if (!std::isnan(hw_joint_command_))
    this->odrive->set_input_torque( hw_joint_command_);

  return hardware_interface::return_type::OK;
}

}  // namespace odrive_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  odrive_hw_interface::OdriveHardware, hardware_interface::ActuatorInterface)