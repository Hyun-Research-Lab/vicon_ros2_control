#ifndef VICON_HARDWARE_INTERFACE_HPP_
#define VICON_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include "vicon_hardware_interface/vicon_tracking_object.hpp"
#include "DataStreamClient.h"

namespace vicon_hardware_interface
{

    using namespace ViconDataStreamSDK::CPP;

    class ViconHardwareInterface : public hardware_interface::SensorInterface
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

    private:
        std::map<std::string, ViconTrackingObject> viconObjects;
        Client viconClient;

        std::string hostname;

        bool connect(std::string hostname, int bufferSize);
        bool disconnect();
        
    };
} // namespace vicon_hardware_interface

#endif // VICON_HARDWARE_INTERFACE_HPP_
