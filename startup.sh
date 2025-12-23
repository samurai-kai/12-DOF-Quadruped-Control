# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # Use Fast DDS (no Zenoh)
cd /home/ws
colcon build --packages-select rl12dof_urdf_description
source install/setup.bash

# Set display for GUI apps (RViz, rqt, Gazebo)
export DISPLAY=host.docker.internal:0.0
export QT_X11_NO_MITSHM=1

echo "ROS2 workspace is ready for you KAI. Environment configured for GUI and Fast DDS."
exec "$@"