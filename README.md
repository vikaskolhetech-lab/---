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

Source Code: import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

# QoS imports
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

# PX4 message imports
from px4_msgs.msg import (
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleLocalPosition,
)

# Detection message
from vision_interfaces.msg import Detection


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        # Publishers
        self.mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # QoS profile (depth=10 for better reliability)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,                     # <-- increased from 1
        )

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.local_position_callback,
            px4_qos,
        )

        # Subscriber for person detection
        self.detection_sub = self.create_subscription(
            Detection,
            '/vision/detections',
            self.detection_callback,
            10
        )

        # Store current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # Target position (setpoints)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -5.0

        # Person detection state
        self.person_detected = False
        self.last_detection_time = self.get_clock().now()

        # Counter for offboard/arm commands
        self.offboard_counter = 0

        # 20 Hz timer
        self.create_timer(0.05, self.timer_callback)

    def local_position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

        self.get_logger().info(
            f"Current Position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}"
        )

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    # ---------- FIXED detection_callback ----------
    def detection_callback(self, msg):
        if msg.class_name != "person":
            return

        # Update timestamp so timer does NOT reset targets
        self.last_detection_time = self.get_clock().now()

        error = msg.center_x - 0.5

        # Move 30 cm in front of current position (no cumulative drift)
        self.target_x = self.current_x + 0.30
        # Lateral correction based on error
        self.target_y = self.current_y + error * 1.5
        self.target_z = -5.0

        self.get_logger().info(
            f"Person detected  "
            f"x={self.current_x:.2f}->{self.target_x:.2f}  "
            f"y={self.current_y:.2f}->{self.target_y:.2f}"
        )

    def timer_callback(self):
        # Offboard control mode
        mode_msg = OffboardControlMode()
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        mode_msg.position = True
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        self.mode_pub.publish(mode_msg)

        # Offboard and arm commands (first 2.5 seconds)
        if self.offboard_counter == 20:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
            )
            self.get_logger().info("Offboard Enabled")

        if self.offboard_counter == 40:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
            )
            self.get_logger().info("Vehicle Armed")

        if self.offboard_counter < 50:
            self.offboard_counter += 1

        # Hover if no detection for >1 second (now works because timestamp is updated)
        now = self.get_clock().now()
        dt = (now - self.last_detection_time).nanoseconds / 1e9
        if dt > 1.0:
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = -5.0
            self.person_detected = False
        # else keep target from detection callback

        # Publish trajectory setpoint
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [self.target_x, self.target_y, self.target_z]
        sp.yaw = 0.0
        self.setpoint_pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
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
