# vicon_ros2_control

Adds a hardware interface which collects, fitlers, and publishes position, orientation, velocity, and angular velocity. Simply clone this repo in the `src` folder of your ros2 workspace. Then, you can add this hardware interface to any of your `<robot_name>.ros2_control.xacro` files. For example:
```xml
<!-- vicon hardware interface (sensor) -->
<ros2_control name="$robot_name_vicon" type="sensor">
<hardware>
    <plugin>vicon_hardware_interface/ViconHardwareInterface</plugin>
    <param name="hostname">192.168.1.112:801</param>
    <param name="update_rate_hz">200</param>
</hardware>
<sensor name="cf01">
    <state_interface name="position_x" />
    <state_interface name="position_y" />
    <state_interface name="position_z" />
    <state_interface name="orientation_qx" />
    <state_interface name="orientation_qy" />
    <state_interface name="orientation_qz" />
    <state_interface name="orientation_qw" />
    <state_interface name="velocity_x" />
    <state_interface name="velocity_y" />
    <state_interface name="velocity_z" />
    <state_interface name="omegab_1" />
    <state_interface name="omegab_2" />
    <state_interface name="omegab_3" />
</sensor>
</ros2_control>
```
You can add more sensors if you'd like. The sensor name should match the Vicon Tracker Object name, in this example, it is `cf01` for crazyflie-01. Be sure to set the `hostname` parameter to the `hostname:port` of the computer running 

## Requirements
We use the [ModernRoboticsCpp library](https://github.com/Le0nX/ModernRoboticsCpp)
```
git clone git@github.com:Le0nX/ModernRoboticsCpp.git
gh pr checkout 29
mkdir build
cd build
cmake ..
make all
sudo make install
```
Note that the `gh pr checkout 29` grabs pull request number 29, which just changes `master` to `main` in the `CMakeLists.txt.googletest` file. You can install the `gh` tool [here](https://cli.github.com/) to run this fancy command, or just manually change the conflicting line of code. Your choice. I recommend adding a `COLCON_IGNORE` file in the `ModernRoboticsCpp` library folder, so that `colcon build` does not accidentally build.

