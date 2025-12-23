colcon build --packages-select rl12dof_urdf_description
source install/setup.bash

ros2 launch rl12dof_urdf_description gazebo_sim.launch.py
