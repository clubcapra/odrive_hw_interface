
#include <limits>
#include <vector>

#include "odrive_hw_interface/odrive_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "odrive-can.hpp"

namespace odrive_hw_interface
{
#define ODRIVE_LOGGER rclcpp::get_logger("OdriveHardware")
#define LOG_AND_RETURN_IF_FATAL(condition, ...)       \
    do                                                \
    {                                                 \
        if (condition)                                \
        {                                             \
            RCLCPP_FATAL(ODRIVE_LOGGER, __VA_ARGS__); \
            return CallbackReturn::ERROR;             \
        }                                             \
    } while (0)

    using hardware_interface::ActuatorInterface;
    using hardware_interface::CallbackReturn;
    using hardware_interface::CommandInterface;
    using hardware_interface::ComponentInfo;
    using hardware_interface::HardwareInfo;
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::InterfaceInfo;
    using hardware_interface::return_type;
    using hardware_interface::StateInterface;

OdriveHardware::OdriveHardware()
    : ActuatorInterface()
{
    this->hw_joint_state_ = std::numeric_limits<double>::quiet_NaN();
    this->hw_joint_command_ = std::numeric_limits<double>::quiet_NaN();
}

CallbackReturn OdriveHardware::on_init(const HardwareInfo &info)
{
    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::SUCCESS;
    }
    RCLCPP_INFO(ODRIVE_LOGGER, "Interface init");

    const ComponentInfo &joint = info_.joints[0];
    const char *jname = joint.name.c_str();
    LOG_AND_RETURN_IF_FATAL(
        joint.command_interfaces.size() != 1,
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        jname, joint.command_interfaces.size());

    auto interface = &joint.command_interfaces[0];
    LOG_AND_RETURN_IF_FATAL(
        interface->name != HW_IF_EFFORT,
        "Joint '%s' have %s command interfaces found. '%s' expected.",
        jname, interface->name.c_str(), HW_IF_EFFORT);

    LOG_AND_RETURN_IF_FATAL(
        joint.state_interfaces.size() != 1,
        "Joint '%s' has %zu state interface. 1 expected.", jname);

    interface = &joint.state_interfaces[0];
    LOG_AND_RETURN_IF_FATAL(
        interface->name != HW_IF_EFFORT,
        "Joint '%s' have %s state interface. '%s' expected.",
        jname, interface->name.c_str(), HW_IF_EFFORT);

    int id = 0;
    char can_if[] = "can0";
    this->odrive = new OdriveCAN(id, can_if);

    return CallbackReturn::SUCCESS;
}

    std::vector<StateInterface> OdriveHardware::export_state_interfaces()
    {
        std::vector<StateInterface> state_interfaces;
        state_interfaces.emplace_back(StateInterface(
            this->info_.joints[0].name, HW_IF_EFFORT, &hw_joint_state_));
        return state_interfaces;
    }

    std::vector<CommandInterface> OdriveHardware::export_command_interfaces()
    {
        std::vector<CommandInterface> command_interfaces;
        command_interfaces.emplace_back(CommandInterface(
            this->info_.joints[0].name, HW_IF_EFFORT, &hw_joint_command_));
        return command_interfaces;
    }

    CallbackReturn OdriveHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {

        // TODO(anyone): prepare the robot to receive commands
        this->odrive->motor_on();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn OdriveHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO(anyone): prepare the robot to stop receiving commands
        this->odrive->motor_off();

        return CallbackReturn::SUCCESS;
    }

    return_type OdriveHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // TODO(anyone): read robot states
        this->odrive->update_motor_state();
        hw_joint_state_ = this->odrive->get_pos();

        return return_type::OK;
    }

    return_type OdriveHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

        if (!std::isnan(hw_joint_command_))
            this->odrive->set_input_torque(hw_joint_command_);

        return return_type::OK;
    }

} // namespace odrive_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    odrive_hw_interface::OdriveHardware, hardware_interface::ActuatorInterface)
