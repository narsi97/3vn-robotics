cmake_minimum_required(VERSION 3.8)
project(threevn_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -Werror on purpose: this is the only code in the project that will
  # eventually move servos, and a warning here is a defect waiting for a
  # bench session.
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(threevn_hardware SHARED
  src/esp32_system_interface.cpp
  src/serial_transport.cpp
)
target_compile_features(threevn_hardware PUBLIC cxx_std_17)
target_include_directories(threevn_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(threevn_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface threevn_hardware.xml)

install(TARGETS threevn_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The protocol needs nothing but a compiler -- no ROS, no device.
  ament_add_gmock(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)

  # The ESP32 firmware's safety logic, compiled natively.
  #
  # The include path reaches into firmware/esp32/include. That dependency
  # is test-only and deliberate: this is where the repository runs C++
  # tests, and the alternative is testing an e-stop by flashing it and
  # trying it on a powered arm.
  # The firmware tree sits beside the ROS workspace, not inside it, so
  # locate it rather than assuming a relative depth.
  get_filename_component(_ws_root "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
  set(_firmware_include "${_ws_root}/firmware/esp32/include")

  if(EXISTS "${_firmware_include}/controller.hpp")
    ament_add_gmock(test_firmware_logic test/test_firmware_logic.cpp)
    target_include_directories(test_firmware_logic PRIVATE
      include
      "${_firmware_include}"
    )
  else()
    message(WARNING
      "firmware sources not found at ${_firmware_include}; "
      "the ESP32 safety-logic tests will NOT run")
  endif()

  # The hardware interface, driven against a fake transport.
  ament_add_gmock(test_system_interface test/test_system_interface.cpp)
  target_link_libraries(test_system_interface threevn_hardware)
  ament_target_dependencies(test_system_interface
    hardware_interface rclcpp rclcpp_lifecycle)
endif()

ament_export_include_directories(include)
ament_export_libraries(threevn_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
