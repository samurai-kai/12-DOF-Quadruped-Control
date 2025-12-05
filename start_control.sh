source /opt/ros/jazzy/setup.bash
colcon build --packages-select rl12dof_urdf_description
source install/setup.bash

# 1. Check controllers
ros2 control list_controllers

ros2 launch rl12dof_urdf_description test_param.launch.py


# ros2 topic pub /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.5, 0.2]}"
