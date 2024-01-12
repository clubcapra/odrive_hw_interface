#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

using hardware_interface::return_type;

class ODriveHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
    return_type configure(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type start() override;

    return_type stop() override;

    return_type read() override;

    return_type write() override;

private:
    // Private member variables for CAN bus communication and motor state
};

#endif  // HARDWARE_INTERFACE_HPP