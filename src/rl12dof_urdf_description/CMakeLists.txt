cmake_minimum_required(VERSION 3.8)
project(rl12dof_urdf_description)

find_package(ament_cmake REQUIRED)

#############################
# Install Python Executables
#############################
install(PROGRAMS
  scripts/joint_test_param_node.py
  DESTINATION lib/${PROJECT_NAME}
)

#############################
# Install URDF / XACRO / Mesh / Config / Launch
#############################

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install URDF/Xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install meshes (if any)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()