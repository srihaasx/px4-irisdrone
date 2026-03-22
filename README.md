# PX4 Iris Drone — ROS 2 SITL Perception Pipeline

Autonomous drone pipeline: takes off, flies toward a red landing pad, detects it with OpenCV HSV color detection, returns above it, and lands precisely on it.

**Stack:** Ubuntu 22.04 · ROS 2 Humble · PX4 v1.14 · Gazebo Classic 11 · Python · OpenCV

## Nodes
- `state_subscriber` — logs position, velocity, attitude, armed state from PX4
- `perception_node` — HSV red detection on downward camera feed
- `waypoint_navigator` — fly → detect → return → land mission logic

## Launch
```bash
# Terminal 1
cd ~/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
PX4_SITL_WORLD=red_boxes make px4_sitl gazebo-classic_iris_opt_flow

# Terminal 2
MicroXRCEAgent udp4 -p 8888

# Terminal 3
ros2 launch assignment_ws demo.launch.py
```
