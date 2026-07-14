# Vision-Based Person Following Drone using ROS 2, PX4 & YOLOv8

An autonomous drone capable of detecting and following a person in simulation using **ROS 2 Jazzy**, **PX4 SITL**, **Gazebo Harmonic**, and **YOLOv8**.

---

# Demo

The drone:

- Takes off in Offboard mode
- Receives camera images from Gazebo
- Detects a person using YOLOv8
- Continuously publishes trajectory setpoints
- Follows the detected person

---

# Tech Stack

- Ubuntu 24.04
- ROS 2 Jazzy
- PX4 Autopilot
- Gazebo Harmonic
- Micro XRCE DDS Agent
- YOLOv8
- Python

---

# Project Structure

```
px4_ros_ws
│
├── src
│   ├── vision_detector
│   │      detector.py
│   │
│   ├── vision_interfaces
│   │      Detection.msg
│   │
│   └── person_follower
│          follower.py
│
└── install
```

---

# ROS Nodes

## 1. Camera Bridge

Bridges Gazebo camera to ROS 2.

Topic

```
/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
```

↓

ROS Image Topic

```
sensor_msgs/Image
```

---

## 2. YOLO Detector

File

```
vision_detector/detector.py
```

Responsibilities

- Subscribe camera image
- Run YOLOv8
- Detect people
- Publish detection message

Published Topic

```
/vision/detections
```

Message

```
vision_interfaces/Detection
```

---

## 3. Person Follower

File

```
person_follower/follower.py
```

Responsibilities

- Subscribe

```
/vision/detections
```

- Subscribe

```
/fmu/out/vehicle_local_position_v1
```

- Compute target position

- Publish

```
/fmu/in/trajectory_setpoint
```

---

# Required Files

```
vision_detector/
    detector.py

person_follower/
    follower.py

vision_interfaces/
    Detection.msg
```

---

# Build Workspace

```
cd ~/px4_ros_ws

source /opt/ros/jazzy/setup.bash

colcon build

source install/setup.bash
```

---

# Complete Startup Guide

## Terminal 1

Start Micro XRCE Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

---

## Terminal 2

Launch PX4

```bash
cd ~/drone_sim/PX4-Autopilot

make px4_sitl gz_x500_mono_cam
```

Wait until Gazebo starts.

---

## Terminal 3

Source ROS

```bash
source /opt/ros/jazzy/setup.bash

source ~/px4_ros_ws/install/setup.bash
```

---

## Terminal 4

Start Gazebo Camera Bridge

```bash
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image
```

---

## Terminal 5

Start YOLO Detector

```bash
source ~/yolo_env/bin/activate

source /opt/ros/jazzy/setup.bash

source ~/px4_ros_ws/install/setup.bash

cd ~/vision_ai

python detector.py
```

Expected output

```
YOLO Detector Started
```

---

## Terminal 6

Start Person Follower

```bash
source /opt/ros/jazzy/setup.bash

source ~/px4_ros_ws/install/setup.bash

ros2 run person_follower follower
```

Expected output

```
Offboard Enabled

Vehicle Armed

Person detected

Publishing trajectory setpoints
```

---

# Spawn Walking Person

Spawn a person into Gazebo.

```bash
gz service -s /world/default/create \
--reqtype gz.msgs.EntityFactory \
--reptype gz.msgs.Boolean \
--timeout 5000 \
--req '
name: "person1"
sdf_filename: "/home/vikas/gazebo_models/walking_person/model.sdf"
pose {
  position {
    x: 5
    y: 0
    z: 0
  }
}'
```

---

# Move the Person

Inside Gazebo

```
View

↓

Transform Control

↓

Select Person

↓

Drag using X Y Z arrows
```

The drone should continuously follow the person.

---

# ROS Topics

Verify camera

```bash
ros2 topic list | grep image
```

Verify detections

```bash
ros2 topic echo /vision/detections
```

Verify trajectory setpoints

```bash
ros2 topic echo /fmu/in/trajectory_setpoint
```

Verify local position

```bash
ros2 topic echo /fmu/out/vehicle_local_position_v1
```

Verify vehicle status

```bash
ros2 topic echo /fmu/out/vehicle_status_v1
```

---

# Common Issues

## Drone Arms but Doesn't Move

Possible causes

- QoS mismatch
- Wrong PX4 topic
- No trajectory setpoints
- Offboard mode not accepted

---

## YOLO Detects but Drone Doesn't Follow

Check

```
ros2 topic echo /vision/detections
```

Should continuously publish

```
class_name: person
```

---

Check trajectory setpoints

```
ros2 topic echo /fmu/in/trajectory_setpoint
```

The target position should continuously change.

---

## No Camera Image

Restart

```bash
ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image
```

---

## Person Not Visible

Spawn again

```bash
gz service -s /world/default/create \
--reqtype gz.msgs.EntityFactory \
--reptype gz.msgs.Boolean \
--timeout 5000 \
--req '
name: "person1"
sdf_filename: "/home/vikas/gazebo_models/walking_person/model.sdf"
pose {
  position {
    x: 5
    y: 0
    z: 0
  }
}'
```

---

# Lessons Learned

- ROS 2 QoS Policies
- PX4 Offboard Control
- Gazebo Simulation
- YOLOv8 Integration
- ROS 2 Message Passing
- Computer Vision for Robotics
- Autonomous Drone Navigation
- Multi-node ROS 2 Architecture
- Debugging PX4 + Gazebo Systems

---

# Future Improvements

- Multi-person tracking
- Obstacle avoidance
- Person re-identification
- MAVSDK integration
- Mission planning using LLMs
- Real drone deployment
- Object tracking with depth estimation

---

# Author

**Vikas Kolhe**

Robotics Engineer

ROS 2 • PX4 • Computer Vision • Autonomous Systems
