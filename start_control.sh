source install/setup.bash

# 1. Check controllers
ros2 control list_controllers

# dont need this anymore since I do this in the launch file
# ros2 control load_controller --set-state active joint_state_broadcaster
# ros2 control load_controller --set-state active joint_group_position_controller
# ros2 control list_controllers

# ros2 topic pub /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.5, 0.2]}"
