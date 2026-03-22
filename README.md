# px4-irisdrone — ROS 2 SITL Perception Pipeline
Autonomous drone simulation built on PX4, ROS 2 Humble, and Gazebo Classic. The drone takes off, flies toward a red landing pad, detects it using a downward-facing camera and OpenCV HSV color segmentation, returns above the pad, and lands precisely on it.

## Prerequisites

| Requirement | Version |
|-------------|---------|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| PX4-Autopilot | v1.14 |
| Gazebo | Classic 11 |
| MicroXRCEAgent | latest |
| Python | 3.10+ |
| OpenCV | 4.x via cv_bridge |

## Demo

[demo video](https://drive.google.com/file/d/1achs2OyJVLYNSaOS5d_oU86xgH0yTWqp/view?usp=sharing)

## Architecture

```
                                ┌──────────────────────────────────────────────────────────────┐
                                │                      Simulation Layer                        │
                                │                                                              │
                                │   PX4 SITL          Gazebo Classic 11       uXRCE-DDS        │
                                │   (autopilot   →    (physics + camera   →   (PX4 ↔ ROS 2     │
                                │    firmware)         simulation)             bridge)         │
                                │                           │                      │           │
                                │                    /camera/image_raw      /fmu/out/*         │
                                └───────────────────────────┼──────────────────────┼───────────┘
                                                            │                      │
                                ┌───────────────────────────▼──────────────────────▼───────────┐
                                │                     ROS 2 Workspace (assignment_ws)          │
                                │                                                              │
                                │  state_subscriber    perception_node     waypoint_navigator  │
                                │  reads PX4 state  →  reads camera     →  reads detection     │
                                │  logs to console     runs HSV detect     flies mission       │
                                │                      publishes result    lands on pad        │
                                │                                                              │
                                │                       rosbag2 recorder                       │
                                │                       records all 6 topics                   │
                                └──────────────────────────────────────────────────────────────┘
---

## File structure

```
px4-irisdrone/
├── src/
│   └── assignment_ws/
│       ├── assignment_ws/
│       │   ├── __init__.py
│       │   ├── state_subscriber.py    # PX4 state logger
│       │   ├── perception_node.py     # OpenCV HSV detection
│       │   └── waypoint_navigator.py  # Mission logic
│       ├── launch/
│       │   └── demo.launch.py         # Starts all 3 nodes
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── bag_info.txt                       # rosbag2 recording evidence
├── assignment.webm                    # Demo video
└── README.md
```

The Gazebo world file with the red landing pad lives at:
```
~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/red_boxes.world
```

## Run

**Terminal 1 — PX4 SITL:**
```bash
cd ~/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash \
  ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
PX4_SITL_WORLD=red_boxes make px4_sitl gazebo-classic_iris_opt_flow
```
Wait for: `Ready for takeoff!`

**Terminal 2 — uXRCE-DDS bridge:**
```bash
MicroXRCEAgent udp4 -p 8888
```
Wait for: `session established`

**Terminal 3 — ROS 2 nodes:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch assignment_ws demo.launch.py
```
Watch for: `Arming` → `Detection 1/5` → `Box confirmed` → `Above box` → `Landed on box`

**Terminal 4 — rosbag (optional):**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/ros2_ws/bags
ros2 bag record -o ~/ros2_ws/bags/flight_01 \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_status \
  /camera/image_raw \
  /perception/image_overlay \
  /perception/detection
```

---


