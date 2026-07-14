Person Follower Drone with PX4, ROS2, Gazebo, and YOLOv8
This repository provides a complete simulation environment for a drone that detects and follows a person using a monocular camera, YOLOv8, and the PX4 autopilot in Gazebo Harmonic. The system is built on ROS 2 Jazzy and uses the Micro XRCE DDS Agent for communication between PX4 and ROS 2.

Overview
Simulation: Gazebo Harmonic with PX4 Software‑In‑The‑Loop (SITL)

Detection: YOLOv8 (Ultralytics) running on simulated camera images

Control: Offboard mode with trajectory setpoints

Communication: ROS 2 via px4_msgs, ros_gz_bridge, and Micro XRCE‑DDS

The drone hovers at a fixed altitude and, upon detecting a person, moves laterally and forward to keep the person centred in the image. If no person is detected for more than 1 second, it reverts to a hover.

System Requirements
Ubuntu 24.04 (or 22.04 with ROS 2 Jazzy)

ROS 2 Jazzy (recommended) – Installation guide

PX4 Autopilot (main branch) – Setup

Gazebo Harmonic – Installation

Micro XRCE DDS Agent – Installation

YOLOv8 – via ultralytics Python package

Python 3.10+ with virtual environment support

Repository Structure
text
person_follower_drone/
├── px4_ros_ws/                  # ROS 2 workspace
│   └── src/
│       ├── person_follower/     # follower controller package
│       │   └── person_follower/
│       │       └── follower.py  # main node
│       └── vision_interfaces/   # custom message for detection
│           └── msg/Detection.msg
├── vision_ai/                   # YOLO detector script
│   └── detector.py
├── README.md
└── setup_instructions.md        # detailed terminal commands
Setup Instructions
1. Install System Dependencies
Install ROS 2 Jazzy (desktop), Gazebo Harmonic, and the Micro XRCE DDS Agent following the official guides.

bash
# Example for Micro XRCE DDS Agent (after installing via apt or building from source)
sudo apt install ros-jazzy-rmw-fastrtps-cpp   # if needed
2. Clone and Build the PX4‑Autopilot
bash
mkdir -p ~/drone_sim
cd ~/drone_sim
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl gz_x500_mono_cam   # builds the simulator with the x500 with monocular camera
3. Create the ROS 2 Workspace
bash
mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src
Clone required packages (or copy your own):

person_follower package (contains the follower node)

vision_interfaces package (defines the Detection message)

Example structure:

bash
git clone <your-person-follower-repo> person_follower
git clone <your-vision-interfaces-repo> vision_interfaces
Build the workspace:

bash
cd ~/px4_ros_ws
colcon build --symlink-install
source install/setup.bash
4. Prepare the Python Environment for YOLO
bash
cd ~
python3 -m venv yolo_env
source yolo_env/bin/activate
pip install ultralytics opencv-python numpy
Create the detector script at ~/vision_ai/detector.py (see section below).

System Architecture
text
+-------------------+       +-------------------+
|   Terminal 1      |       |   Terminal 2      |
| MicroXRCE Agent   |<----->| PX4 SITL + Gazebo |
| (UDP port 8888)   |       | (model with cam)  |
+-------------------+       +-------------------+
            ^                         ^
            | (DDS)                   | (Gazebo Transport)
            v                         v
+-------------------+       +-------------------+
|   Terminal 4      |       |   Terminal 5      |
| ros_gz_bridge     |------>| YOLO detector     |
| (camera image)    |       | (publishes /vision/detections)
+-------------------+       +-------------------+
            ^                         |
            |                         v
            |               +-------------------+
            +-------------->|   Terminal 6      |
                            | follower.py       |
                            | (offboard control)|
                            +-------------------+
MicroXRCEAgent bridges PX4 and ROS 2 over UDP.

PX4 SITL publishes vehicle state and receives setpoints.

Gazebo streams camera images via ros_gz_bridge.

YOLO detector processes images and publishes Detection messages.

Follower node subscribes to detections and sends trajectory setpoints to PX4.

Running the System
Open six separate terminals and run the following commands.

Terminal 1 – Micro XRCE DDS Agent
bash
MicroXRCEAgent udp4 -p 8888
Expected output:

text
Running...
UDP transport: 8888
Terminal 2 – PX4 SITL + Gazebo
bash
cd ~/drone_sim/PX4-Autopilot
make px4_sitl gz_x500_mono_cam
Wait until Gazebo opens and the drone appears. PX4 will display a boot log and finally show INFO [commander] Ready for takeoff!.

Terminal 3 – Source ROS Workspace (optional debug)
bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros_ws/install/setup.bash
Keep this terminal for ros2 topic list, echo, etc.

Terminal 4 – Camera Bridge (ros_gz_bridge)
bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image
Verify the topic exists:

bash
ros2 topic list | grep image
# Should show: /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
Terminal 5 – YOLO Person Detector
bash
source ~/yolo_env/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/px4_ros_ws/install/setup.bash
cd ~/vision_ai
python3 detector.py
The detector subscribes to the camera image, runs YOLOv8, and publishes vision_interfaces/msg/Detection on topic /vision/detections when a person is detected (class ID 0).

Terminal 6 – Person Follower Controller
bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros_ws/install/setup.bash
ros2 run person_follower follower
The node will:

Wait 2.5 seconds (50 timer ticks) to switch to offboard mode and arm.

Continuously publish setpoints (hover at z = -5.0).

When a person is detected, adjust x (forward 0.3 m) and y (lateral correction).

If no detection for >1 second, return to hover.

Code of Follower:

import rclpy
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

Notes:

The drone flies forward relative to its current position, not accumulating drift.

The detection timestamp is updated on each detection, preventing premature hover reset.

Default altitude is -5.0 m (Gazebo uses NED, so negative = up).

detector.py (example outline)
python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from vision_interfaces.msg import Detection

class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.sub = self.create_subscription(Image, '/world/.../image', self.callback, 10)
        self.pub = self.create_publisher(Detection, '/vision/detections', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_img)
        # Extract person detections (class 0)
        for box in results[0].boxes:
            if box.cls == 0:
                x1, y1, x2, y2 = box.xyxy[0]
                center_x = (x1 + x2) / 2 / cv_img.shape[1]   # normalize 0..1
                center_y = (y1 + y2) / 2 / cv_img.shape[0]
                detection = Detection()
                detection.class_name = 'person'
                detection.center_x = center_x
                detection.center_y = center_y
                self.pub.publish(detection)
Custom Messages
The vision_interfaces package defines Detection.msg:

text
string class_name
float32 center_x
float32 center_y
float32 width
float32 height
Make sure this package is built in your workspace.

Testing & Verification
Check if the drone arms and takes off to hover at z=-5.0.

Place a person (or another object) in the Gazebo world (e.g., using the insert menu) and walk around.

The drone should move to follow the person in x and y.

If the person disappears, the drone should stop and hover after 1 second.

Use ros2 topic echo /fmu/out/vehicle_local_position_v1 and ros2 topic echo /vision/detections to monitor data.

Troubleshooting
Issue	Solution
MicroXRCEAgent fails to start	Check that port 8888 is free. Use sudo lsof -i:8888.
Gazebo does not open	Install Gazebo Harmonic properly. Check make px4_sitl gz_x500_mono_cam output.
No camera image topic	Ensure the bridge is running with the correct topic path (double‑check Gazebo model name).
Drone does not arm	Wait for the 2.5‑second initialisation. Check that offboard mode is enabled.
YOLO detection not publishing	Verify the detector subscribes to the correct image topic and that cv_bridge works.
vision_interfaces not found	Build the workspace and source install/setup.bash.
Drone drifts indefinitely	Check that the detection callback is actually updating the target. Enable debug logs in follower.py.
Acknowledgements
This project uses:

PX4 Autopilot

ROS 2

Gazebo

Ultralytics YOLOv8

