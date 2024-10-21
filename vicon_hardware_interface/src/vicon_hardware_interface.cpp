#include "vicon_hardware_interface/vicon_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vicon_hardware_interface
{
hardware_interface::CallbackReturn ViconHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize mocap data
  position_x_ = 0.0;
  position_y_ = 0.0;
  position_z_ = 0.0;
  orientation_roll_ = 0.0;
  orientation_pitch_ = 0.0;
  orientation_yaw_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ViconHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Expose position state interfaces
  state_interfaces.emplace_back("vicon_object", "position_x", &position_x_);
  state_interfaces.emplace_back("vicon_object", "position_y", &position_y_);
  state_interfaces.emplace_back("vicon_object", "position_z", &position_z_);

  // Expose orientation state interfaces
  state_interfaces.emplace_back("vicon_object", "orientation_roll", &orientation_roll_);
  state_interfaces.emplace_back("vicon_object", "orientation_pitch", &orientation_pitch_);
  state_interfaces.emplace_back("vicon_object", "orientation_yaw", &orientation_yaw_);

  return state_interfaces;
}

hardware_interface::CallbackReturn ViconHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // Perform any startup tasks
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ViconHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Perform any shutdown tasks
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ViconHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Read mocap data and update the state variables
  // This is where you'd fetch data from the Vicon system
  position_x_ = 1.0;
  position_y_ = 2.0;
  position_z_ = 3.0;
  orientation_roll_ = 4.0;
  orientation_pitch_ = 5.0;
  orientation_yaw_ = 6.0;

  return hardware_interface::return_type::OK;
}

  hardware_interface::return_type ViconHardwareInterface::write(
        const rclcpp::Time &, const rclcpp::Duration &)
  {
    // Since this is a read-only interface, return OK without any action.
    return hardware_interface::return_type::OK;
  }
}  // namespace vicon_hardware_interface

// register plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
	vicon_hardware_interface::ViconHardwareInterface, hardware_interface::SystemInterface)
