#include "vicon_hardware_interface/vicon_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <ctime>

namespace vicon_hardware_interface
{
  hardware_interface::CallbackReturn ViconHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // info_.hardware_interface
    std::string name = info_.sensors[0].name;
    RCLCPP_INFO(get_logger(), "Our sensor name is %s", name.c_str());

    // set the hostname
    hostname = info_.hardware_parameters["hostname"];
    updateRateHz = std::stoi(info_.hardware_parameters["update_rate_hz"]);

    // create all of the tracking objects
    for (const auto &sensor : info_.sensors)
    {
      if (viconTrackingObject.AddSensor(sensor.name)) {
        outputStates[sensor.name] = FullState();
      }
      else {
        RCLCPP_ERROR(this->get_logger(), "Sensor %s already exists", sensor.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(this->get_logger(), "ViconHardwareInterface initialized successfully.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ViconHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // print hello
    RCLCPP_INFO(this->get_logger(), "Exporting state interfaces...");
    for (auto name : viconTrackingObject.GetSensorNames())
    {
      state_interfaces.emplace_back(name, "position_x", (double *)&(outputStates[name].position_x));
      state_interfaces.emplace_back(name, "position_y", (double *)&(outputStates[name].position_y));
      state_interfaces.emplace_back(name, "position_z", (double *)&(outputStates[name].position_z));

      state_interfaces.emplace_back(name, "velocity_x", (double *)&(outputStates[name].velocity_x));
      state_interfaces.emplace_back(name, "velocity_y", (double *)&(outputStates[name].velocity_y));
      state_interfaces.emplace_back(name, "velocity_z", (double *)&(outputStates[name].velocity_z));

      state_interfaces.emplace_back(name, "orientation_qw", (double *)&(outputStates[name].orientation_qw));
      state_interfaces.emplace_back(name, "orientation_qx", (double *)&(outputStates[name].orientation_qx));
      state_interfaces.emplace_back(name, "orientation_qy", (double *)&(outputStates[name].orientation_qy));
      state_interfaces.emplace_back(name, "orientation_qz", (double *)&(outputStates[name].orientation_qz));

      state_interfaces.emplace_back(name, "omegab_1", (double *)&(outputStates[name].omegab_1));
      state_interfaces.emplace_back(name, "omegab_2", (double *)&(outputStates[name].omegab_2));
      state_interfaces.emplace_back(name, "omegab_3", (double *)&(outputStates[name].omegab_3));
    }
    RCLCPP_INFO(this->get_logger(), "Exported %zu state interfaces", state_interfaces.size());

    return state_interfaces;
  }

  hardware_interface::CallbackReturn ViconHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    // Perform any startup tasks
    if (!connect(200))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    timer_ = node_->create_wall_timer(
      std::chrono::microseconds(static_cast<int>(1e6 / updateRateHz)),
      std::bind(&ViconHardwareInterface::readViconFrame, this));

    spin_thread_ = std::thread([this]() {
      rclcpp::spin(node_);
    });

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ViconHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    // Perform any shutdown tasks
    if (!disconnect())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ViconHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // Read mocap data and update the state variables
    // This is where you'd fetch data from the Vicon system
    // RCLCPP_INFO(this->get_logger(), "Reading data from Vicon system...");
    for (const std::string &sensorName : viconTrackingObject.GetSensorNames())
    {
      outputStates[sensorName] = viconTrackingObject.GetLatestState(sensorName);
    }

    // getFrame();
    // RCLCPP_INFO(this->get_logger(), "Data read successfully.");
    return hardware_interface::return_type::OK;
  }

  // Connect to the Vicon system
  bool ViconHardwareInterface::connect(int bufferSize)
  {
    std::string msg;
    int counter = 0;

    msg = "Attempting to connect to " + hostname + "...";
    RCLCPP_INFO(this->get_logger(), msg.c_str());

    while (!viconClient.IsConnected().Connected)
    {
      bool ok = (viconClient.Connect(hostname).Result == Result::Success);
      if (!ok)
      {
        counter++;
        msg = "Connect failed, reconnecting (" + std::to_string(counter) + ")...";
        RCLCPP_INFO(this->get_logger(), msg.c_str());
        sleep(1);
      }

      // Check if we have exceeded the maximum number of attempts
      if (counter > 10)
      {
        msg = "Failed to connect to " + hostname + " after 10 attempts.";
        RCLCPP_ERROR(this->get_logger(), msg.c_str());
        return false; // Return false if connection fails
      }
    }
    msg = "Connection successfully established with " + hostname;
    RCLCPP_INFO(this->get_logger(), msg.c_str());

    // perform further initialization
    viconClient.EnableSegmentData();
    viconClient.EnableMarkerData();
    viconClient.EnableUnlabeledMarkerData();
    viconClient.EnableMarkerRayData();
    viconClient.EnableDeviceData();
    viconClient.EnableDebugData();

    viconClient.SetStreamMode(StreamMode::ClientPull);
    viconClient.SetBufferSize(bufferSize);

    msg = "Initialization complete";
    RCLCPP_INFO(this->get_logger(), msg.c_str());
    return true; // Return true if successful
  }

  bool ViconHardwareInterface::disconnect()
  {
    if (!viconClient.IsConnected().Connected)
    {
      return true;
    }
    sleep(1);
    viconClient.DisableSegmentData();
    viconClient.DisableMarkerData();
    viconClient.DisableUnlabeledMarkerData();
    viconClient.DisableDeviceData();
    viconClient.DisableCentroidData();

    std::string msg = "Disconnecting from " + hostname + "...";
    RCLCPP_INFO(this->get_logger(), msg.c_str());

    viconClient.Disconnect();
    msg = "Successfully disconnected";
    RCLCPP_INFO(this->get_logger(), msg.c_str());

    if (!viconClient.IsConnected().Connected)
    {
      return true;
    }
    return false;
  }

  bool ViconHardwareInterface::readViconFrame()
  {
    if (viconClient.GetFrame().Result != Result::Success)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to get frame");
      return false;
    }

    // int frame_number = viconClient.GetFrameNumber().FrameNumber;

    unsigned int SubjectCount = viconClient.GetSubjectCount().SubjectCount;
    for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
    {

      // record the global position and orientation
      float x, y, z, qx, qy, qz, qw;
      std::string SubjectName = viconClient.GetSubjectName(SubjectIndex).SubjectName;

      if (viconTrackingObject.HasSensorWithName(SubjectName) == false)
      {
        RCLCPP_ERROR(this->get_logger(), "Subject: %s, not in the map", SubjectName.c_str());
        continue; // Skip if the subject is not in the map
      }

      // Count the number of segments
      unsigned int SegmentCount = viconClient.GetSegmentCount(SubjectName).SegmentCount;

      if (SegmentCount == 0)
      {
        continue; // Skip if there are no segments
      }

      unsigned int SegmentIndex = 0;
      // Get the segment name
      std::string SegmentName = viconClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;

      // Get the global segment translation
      Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation =
          viconClient.GetSegmentGlobalTranslation(SubjectName, SegmentName);

      x = _Output_GetSegmentGlobalTranslation.Translation[0] / 1e3;
      y = _Output_GetSegmentGlobalTranslation.Translation[1] / 1e3;
      z = _Output_GetSegmentGlobalTranslation.Translation[2] / 1e3;

      // Get the global segment rotation in quaternion coordinates
      Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion =
          viconClient.GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName);

      qw = _Output_GetSegmentGlobalRotationQuaternion.Rotation[3];
      qx = _Output_GetSegmentGlobalRotationQuaternion.Rotation[0];
      qy = _Output_GetSegmentGlobalRotationQuaternion.Rotation[1];
      qz = _Output_GetSegmentGlobalRotationQuaternion.Rotation[2];

      RCLCPP_INFO(this->get_logger(), "Subject: %s, Segment: %s, Position: (%f, %f, %f), Orientation: (%f, %f, %f, %f)", SubjectName.c_str(), SegmentName.c_str(), x, y, z, qx, qy, qz, qw);

      // Get the current time point from the system clock
      auto now = std::chrono::system_clock::now();

      // Get the duration since the epoch
      auto duration = now.time_since_epoch();

      // Convert duration to seconds as a double
      double seconds_since_epoch = std::chrono::duration<double>(duration).count();

      HalfState hs;
      hs.time = seconds_since_epoch; // Assuming this is the correct way to get time
      hs.position_x = x;
      hs.position_y = y;
      hs.position_z = z;
      hs.orientation_qw = qw;
      hs.orientation_qx = qx;
      hs.orientation_qy = qy;
      hs.orientation_qz = qz;

      // Push the data into the buffer for the corresponding object
      // This will handle filtering and data storage for us
      viconTrackingObject.PushData(SubjectName, hs);
    }
    return true;
  }
} // namespace vicon_hardware_interface

// register plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    vicon_hardware_interface::ViconHardwareInterface, hardware_interface::SensorInterface)
