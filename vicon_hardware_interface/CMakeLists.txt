cmake_minimum_required(VERSION 3.8)
project(vicon_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ros2 control dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(${EIGEN3_INCLUDE_DIR})

##COMPILE
add_library(
  vicon_hardware_interface
  SHARED
  src/vicon_hardware_interface.cpp
  src/vicon_tracking_object.cpp
)

set(VICON_SDK_DIR ${PROJECT_SOURCE_DIR}/../vicon_sdk)
include_directories(${VICON_SDK_DIR}/include)

target_link_libraries(
  vicon_hardware_interface PUBLIC
  ${VICON_SDK_DIR}/lib/libViconDataStreamSDK_CPP.so
)

target_compile_features(vicon_hardware_interface PUBLIC cxx_std_17)
target_include_directories(vicon_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/vicon_hardware_interface>
)
ament_target_dependencies(
  vicon_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# INSTALL
install(
  DIRECTORY include/ ${VICON_SDK_DIR}/include/
  DESTINATION include
)
install(TARGETS vicon_hardware_interface
  EXPORT export_vicon_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install libraries
install(DIRECTORY ${VICON_SDK_DIR}/lib/
    DESTINATION lib
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface vicon_hardware_interface.xml)

## EXPORTS
ament_export_targets(export_vicon_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
