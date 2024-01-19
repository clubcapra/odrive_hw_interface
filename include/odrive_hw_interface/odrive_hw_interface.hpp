#ifndef ODRIVE_HW_INTERFACE__ODRIVE_HW_INTERFACE_HPP_
#define ODRIVE_HW_INTERFACE__ODRIVE_HW_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>


#include <hardware_interface/hardware_info.hpp>
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <rclcpp/rclcpp.hpp>

#include "odrive-can.hpp"

using hardware_interface::return_type;

namespace odrive_hw_interface {

    class OdriveHardware : public hardware_interface::ActuatorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(OdriveHardware)

        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

     private:
        // Store the command for the simulated robot
        double hw_joint_command_;
        double hw_joint_state_;

        OdriveCAN *odrive; 

    };

}  // namespace odrive_hw_interface

#endif  // ODRIVE_HW_INTERFACE__ODRIVE_HW_INTERFACE_HPP_
