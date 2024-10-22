#include "vicon_hardware_interface/vicon_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vicon_hardware_interface
{
hardware_interface::CallbackReturn ViconHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // info_.hardware_interface
  std::string name = info_.sensors[0].name;
  RCLCPP_INFO(get_logger(), "Our sensor name is %s", name.c_str());

  // create all of the tracking objects
  for (const auto& sensor : info_.sensors) {
    viconObjects[sensor.name] = ViconTrackingObject();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ViconHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto it = viconObjects.begin(); it != viconObjects.end(); ++it) {
    std::string sensorName = it->first;
    ViconTrackingObject obj = it->second;
    const FullState &state = obj.GetOutputState();

    state_interfaces.emplace_back(sensorName, "position_x", (double *)&(state.position_x));
    state_interfaces.emplace_back(sensorName, "position_y", (double *)&(state.position_y));
    state_interfaces.emplace_back(sensorName, "position_z", (double *)&(state.position_z));

    state_interfaces.emplace_back(sensorName, "velocity_x", (double *)&(state.velocity_x));
    state_interfaces.emplace_back(sensorName, "velocity_y", (double *)&(state.velocity_y));
    state_interfaces.emplace_back(sensorName, "velocity_z", (double *)&(state.velocity_z));

    state_interfaces.emplace_back(sensorName, "orientation_qx", (double *)&(state.orientation_qx));
    state_interfaces.emplace_back(sensorName, "orientation_qy", (double *)&(state.orientation_qy));
    state_interfaces.emplace_back(sensorName, "orientation_qz", (double *)&(state.orientation_qz));
    state_interfaces.emplace_back(sensorName, "orientation_qw", (double *)&(state.orientation_qw));

    state_interfaces.emplace_back(sensorName, "angular_velocity_qx_dot", (double *)&(state.angular_velocity_qx_dot));
    state_interfaces.emplace_back(sensorName, "angular_velocity_qy_dot", (double *)&(state.angular_velocity_qy_dot));
    state_interfaces.emplace_back(sensorName, "angular_velocity_qz_dot", (double *)&(state.angular_velocity_qz_dot));
    state_interfaces.emplace_back(sensorName, "angular_velocity_qw_dot", (double *)&(state.angular_velocity_qw_dot));
  }

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

  return hardware_interface::return_type::OK;
}

}  // namespace vicon_hardware_interface

// register plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
	vicon_hardware_interface::ViconHardwareInterface, hardware_interface::SensorInterface)
