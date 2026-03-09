source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 launch lerobot hw_read.launch.py &
ros2 topic echo /joint_states
