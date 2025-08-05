# ArUco Docking Refactor

This project demonstrates an autonomous docking and undocking system using ArUco marker-based vision, ROS2, and MQTT communication.  
It is a modular refactoring of a working system implemented on a real robot.

---

## Features

- ArUco-based marker detection and pose estimation
- Pitch-based alignment (Stage 1)
- X-offset-based realignment
- Z-distance-based curved reverse docking (Stage 2)
- Marker-based or unconditional undocking
- MQTT communication to ROS2 `/cmd_vel` bridge
- Fully modular and testable Python code

---

## Project Structure

aruco_docking_refactor/
├── main.py
├── requirements.txt
├── README.md
└── docking_controller/
├── init.py
├── mqtt_publisher.py
├── aruco_detector.py
├── alignment_controller.py
├── first_stage.py
├── second_stage.py
└── undocking.py



## System Architecture

[Windows PC]

Runs camera (cv2)

ArUco detection & control logic

Publishes via MQTT


 ↓ MQTT
[Linux PC with ROS2]

Receives MQTT messages

Converts to /cmd_vel

Sends to motor controller



---

## Usage

1. Install dependencies:

```bash
pip install -r requirements.txt
Run the full docking → undocking process:


python main.py
Adjust the target_marker_id and req_id in main.py as needed.

Dependencies
opencv-python

numpy

paho-mqtt

Notes
The camera is connected to the Windows PC.

The /cmd_vel topic must be subscribed by a ROS2 node on the Linux PC.

Tested with ArUco DICT_6X6_250, marker size: 93 mm.

Author
Created by a robotics developer with real-world ROS2 docking experience.
This project is a modular, clean refactor of a working production robot system.