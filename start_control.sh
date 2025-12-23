source /opt/ros/jazzy/setup.bash
colcon build --packages-select rl12dof_urdf_description
source install/setup.bash

# Check controllers
ros2 control list_controllers

ros2 launch rl12dof_urdf_description FKTest.launch.py
# ros2 launch rl12dof_urdf_description IKTest.launch.py

#==========================
# for impedance test
ros2 control set_controller_state joint_group_position_controller inactive
ros2 control list_controllers
ros2 control load_controller joint_group_effort_controller --set-state active
ros2 control list_controllers
ros2 launch rl12dof_urdf_description impedance_test.launch.py






# ros2 topic pub /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.5, 0.2]}"
