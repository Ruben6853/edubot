#!/usr/bin/env bash
cd /mnt/c/Users/Wesse/OneDrive/Documents/GitHub/edubot/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lerobot sim_position.launch.py &
SIM_PID=$!

echo "Waiting for /joint_states..."
for i in $(seq 1 30); do
    sleep 1
    if ros2 topic list 2>/dev/null | grep -q "/joint_states"; then
        echo "Found after ${i}s"
        break
    fi
done

sleep 2
echo "--- Running controller ---"
timeout 20 ros2 run controllers_new pos_traj 2>&1
kill $SIM_PID 2>/dev/null
