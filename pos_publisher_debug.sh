source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 run --prefix 'gdb -ex run --args' controllers_new pos_traj