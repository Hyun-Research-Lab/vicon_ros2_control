#ifndef VICON_HARDWARE_INTERFACE_HPP_
#define VICON_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vicon_hardware_interface
{
    class ViconHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        // Configure the hardware interface
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        // Export state interfaces (read-only)
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // Start the hardware interface
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        // Stop the hardware interface
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // Read data from hardware (e.g., mocap system)
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // Variables to hold the mocap data
        double position_x_, position_y_, position_z_;
        double orientation_roll_, orientation_pitch_, orientation_yaw_;
    };
} // namespace vicon_hardware_interface

#endif // VICON_HARDWARE_INTERFACE_HPP_
