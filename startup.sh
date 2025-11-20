# sudo chmod a+rw /dev/ttyACM0
# sudo chmod a+rw /dev/ttyUSB0
# sudo chmod a+rw /dev/i2c-7
source /opt/ros/jazzy/setup.bash
# export RMW_IMPLEMENTATION=rmw_zenoh_cpp
cd /home/ws
colcon build
source install/setup.bash
export DISPLAY=host.docker.internal:0.0
export QT_X11_NO_MITSHM=1
