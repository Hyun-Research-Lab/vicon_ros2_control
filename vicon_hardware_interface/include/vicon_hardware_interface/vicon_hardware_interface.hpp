#ifndef VICON_HARDWARE_INTERFACE_HPP_
#define VICON_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include "vicon_hardware_interface/vicon_tracking_object.hpp"
#include "DataStreamClient.h"
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

namespace vicon_hardware_interface
{

    using namespace ViconDataStreamSDK::CPP;

    struct LoggingStruct {
        HalfState hs_raw;
        HalfState hs_updated;
        FullState fs;
    };

    class ViconHardwareInterface : public hardware_interface::SensorInterface
    {
    public:

        ViconHardwareInterface() : node_(rclcpp::Node::make_shared("vicon_hw_interface_timer")) {}

        ~ViconHardwareInterface();

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
        ViconTrackingObject viconTrackingObject;
        Client viconClient;

        // this is what is exposed to the controller_manager
        std::map<std::string, FullState> outputStates;
        
        std::string hostname;
        int updateRateHz;

        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::thread spin_thread_;
        std::mutex mtx;

        bool connect(int bufferSize);
        bool disconnect();
        bool readViconFrame();

        // for testing only
        std::vector<LoggingStruct> log;

        
    };
} // namespace vicon_hardware_interface

#endif // VICON_HARDWARE_INTERFACE_HPP_
