source /opt/ros/jazzy/setup.bash
colcon build --packages-select rl12dof_urdf_description
source install/setup.bash

# --------------------------
# Sanity check
ros2 control list_controllers

# --------------------------
# Ensure joint_state_broadcaster is ACTIVE
ros2 run controller_manager spawner joint_state_broadcaster

ros2 control list_controllers

# ==========================
# FK test (position control)
ros2 launch rl12dof_urdf_description FKTest.launch.py
sleep 2

# ==========================
# Switch to impedance mode

# Stop position controller
ros2 control set_controller_state joint_group_position_controller inactive

# Start effort controller (use spawner!)
ros2 run controller_manager spawner joint_group_effort_controller

ros2 control list_controllers

# ==========================
# Impedance test
ros2 launch rl12dof_urdf_description impedance_test.launch.py
