source install/setup.bash

# 1. Check controllers
ros2 control list_controllers

# 2. Load + activate the broadcaster
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active position_controller

# 3. Load your control method (pick ONE)
# ros2 control load_controller forward_position_controller

# 4. Activate it
# ros2 control switch_controllers --activate forward_position_controller

# ros2 control switch_controllers \
#     --activate forward_position_controller

ros2 control list_controllers