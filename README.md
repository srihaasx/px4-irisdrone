# px4-irisdrone — ROS 2 SITL Perception Pipeline

Autonomous drone simulation built on PX4, ROS 2 Humble, and Gazebo Classic. The drone takes off, flies toward a red landing pad, detects it using a downward-facing camera and OpenCV HSV color segmentation, returns above the pad, and lands precisely on it.

---

## Demo

demo video

---

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
```

### Topic map

| Topic | Type | Flow |
|-------|------|------|
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4 → state_subscriber, waypoint_navigator |
| `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | PX4 → state_subscriber |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | PX4 → state_subscriber, waypoint_navigator |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → perception_node |
| `/perception/detection` | `std_msgs/String` | perception_node → waypoint_navigator |
| `/perception/image_overlay` | `sensor_msgs/Image` | perception_node → rosbag |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | waypoint_navigator → PX4 |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | waypoint_navigator → PX4 |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | waypoint_navigator → PX4 |

---

## Node descriptions

### `state_subscriber.py`
Subscribes to all PX4 state topics and logs them to the terminal. Uses state-change deduplication on `vehicle_status` — only logs when `arming_state` or `nav_state` actually changes. Purely observational, sends no commands.

### `perception_node.py`
Subscribes to `/camera/image_raw`. On each frame:
1. Converts ROS Image to OpenCV BGR via CvBridge
2. Converts BGR to HSV color space
3. Applies dual-range red mask (hue 0-15 and 160-180, red wraps in HSV)
4. Cleans noise with morphological open/close operations
5. Finds contours and filters by minimum area (500px)
6. Draws bounding boxes on overlay image
7. Publishes `DETECTED` or `NO_DETECTION` to `/perception/detection`
8. Publishes annotated frame to `/perception/image_overlay`

### `waypoint_navigator.py`
Mission controller running at 10Hz. Three sequential phases:

**Phase 1 — Fly forward**
Streams `OffboardControlMode` and `TrajectorySetpoint` at 10Hz. After 1 second switches to offboard mode, arms, then sends setpoint `[20.0, 0.0, -3.0]` in NED (fly north at 3m altitude).

**Phase 2 — Detect and confirm**
Each `DETECTED` message increments a counter and updates saved coordinates. After 5 detections the position is confirmed and the drone returns above `[box_x, box_y, -3.0]`.

**Phase 3 — Descend**
Increments z setpoint by 0.02m per tick (0.2 m/s). Holds x,y fixed at box coordinates. Logs `Landed on box` when z reaches 0.0.

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

---

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

---

## One-time setup

**1. Build px4_msgs matching PX4 firmware version:**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs && git checkout release/1.14
cd ~/ros2_ws && colcon build --packages-select px4_msgs
```

**2. Clone this repo and build:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/srihaasx/px4-irisdrone.git
cd ~/ros2_ws && colcon build --packages-select assignment_ws
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

**3. Create the Gazebo world file:**
```bash
cat > ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/red_boxes.world << 'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <model name="red_box">
      <static>true</static>
      <pose>0 8 0 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>1 1 0.1</size></box></geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
EOF
```

---

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

## Rosbag evidence

See `bag_info.txt`. Key stats:

```
Duration:   67 seconds       Messages: 25071

/fmu/out/vehicle_local_position   7080 msgs   ~105 Hz
/fmu/out/vehicle_attitude        12804 msgs   ~190 Hz
/fmu/out/vehicle_status            114 msgs   on state change only
/camera/image_raw                 1975 msgs    ~29 Hz
/perception/detection             1549 msgs    ~23 Hz
/perception/image_overlay         1549 msgs    ~23 Hz
```

---

## Key design decisions

**VOLATILE QoS** — PX4 publishes all `/fmu/out/*` topics with VOLATILE durability. Using TRANSIENT_LOCAL causes a silent incompatibility — zero data, zero errors. All subscribers use VOLATILE.

**HSV over RGB** — HSV separates color from brightness. Red requires two hue ranges (0-15 and 160-180) because it wraps around the hue spectrum.

**5-detection confirmation** — First detection fires at an angle before the drone is directly above. Waiting for 5 detections and updating coordinates each time gives much more accurate landing position.

**Position control landing** — `VEHICLE_CMD_NAV_LAND` causes bouncing in SITL. Slowly incrementing z setpoint at 0.2 m/s gives smooth touchdown.

**NED vs Gazebo ENU** — Gazebo uses ENU (X=East, Y=North). PX4 uses NED (X=North, Y=East). Pad at Gazebo `(0, 8)` = NED `(8, 0)` — directly in the drone's northward path.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `bind error port 8888` | `sudo docker stop $(sudo docker ps -q)` |
| No `/fmu` topics | Check terminal 2 shows `session established` |
| `NO_DETECTION` always | Lower area threshold in perception_node.py |
| Drone does not arm | Wait for full PX4 boot before launching nodes |
| Gazebo freezes | Close all other apps, use minimal world |
| `unknown target gz_x500` | Use `gazebo-classic_iris_opt_flow` not `gz_x500` |
