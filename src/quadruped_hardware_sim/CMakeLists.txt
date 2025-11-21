cmake_minimum_required(VERSION 3.12)
project(quadruped_hardware_sim)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories
include_directories(
  include
)

# Build the simulation hardware plugin
add_library(${PROJECT_NAME} SHARED
  src/quadruped_hardware_sim.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface quadruped_hardware_sim.xml)

# Install targets and headers
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin XML
install(
  FILES quadruped_hardware_sim.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp hardware_interface pluginlib)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
