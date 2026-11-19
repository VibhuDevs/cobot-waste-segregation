#!/bin/bash
# Single script to launch the cobot with Gazebo and ROS 2 Controllers

cleanup() {
    echo "Stopping the simulation launch..."
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

echo "Launching Gazebo simulation..."
ros2 launch cobot_gazebo cobot.gazebo.launch.py \
    load_controllers:=true \
    world_file:=empty.world \
    use_camera:=true \
    use_rviz:=true \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.03 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0
