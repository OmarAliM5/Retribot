# 🤖 Retribot

### ROS2-Based Autonomous Mobile Robot

## Overview

Retribot is a ROS2-based autonomous mobile robot designed to demonstrate modular robotics software architecture, autonomous navigation, computer vision, and embedded system integration.

The project combines real-time perception, motion control, sensor fusion, and a web-based monitoring interface to enable reliable autonomous operation on a mecanum-wheel robotic platform. Built using a modular ROS2 architecture, each subsystem operates independently while communicating through ROS2 topics and services, providing a scalable and maintainable robotics framework.

---

## ✨ Features

* Autonomous navigation
* Mecanum wheel omnidirectional drive
* Real-time computer vision
* Obstacle detection and avoidance
* IMU-based orientation estimation
* PID-based motion control
* Modular ROS2 package architecture
* Web-based robot monitoring dashboard
* ESP32-S3 microcontroller integration
* Real-time sensor communication

---

## 📂 Repository Structure

```text
Retribot/
│
├── camera_interfaces/        # Custom ROS2 message definitions
├── camera_pkg/               # Camera streaming and computer vision
├── imu_pkg/                  # IMU interface
├── mecanum_controller_pkg/   # Mecanum wheel controller
├── navigation_pkg/           # Autonomous navigation
├── obstacle_sensors/         # TOF obstacle detection
├── process_manager_pkg/      # Node and process management
├── robot_gui/                # Web-based dashboard
└── README.md
```

---

## 🏗️ System Architecture

The robot software follows a modular ROS2 architecture where each package is responsible for a specific subsystem.

### Main Components

* Camera acquisition and perception
* Computer vision processing
* Autonomous navigation
* Robot motion control
* Obstacle detection
* IMU data acquisition
* Robot GUI
* Process management
* Custom ROS2 interfaces

---

## ⚙️ Technologies Used

* ROS2
* Python
* OpenCV
* HTML
* CSS
* JavaScript
* Ubuntu Linux
* ESP32-S3
* CMake

---

## 🤖 Hardware Platform

* Raspberry Pi
* ESP32-S3
* Raspberry Pi Professional Camera (5MP - 200°FOV)
* IMU 
* [ (3) TOF + (4) IR ] Sensors
* Mecanum Wheel Mobile Robot

---

## 🔌 ESP32-S3 Firmware

The embedded firmware responsible for low-level robot control is maintained in a separate repository.

The firmware handles:

* Motor driver control
* Encoder processing
* PID-based wheel control
* IMU communication
* Sensor interfacing
* Serial communication with the ROS2 host

**Repository:** https://github.com/Notsuperman7/Mobile_robot

## 🚀 Main Capabilities

* Real-time camera streaming
* Computer vision processing
* Autonomous navigation
* Obstacle detection
* Robot motion control
* Sensor fusion
* Web-based monitoring dashboard
* ROS2 modular communication

---

## 🔨 Build

Clone the repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/OmarAliM5/Retribot.git
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## ▶️ Running

Launch the required ROS2 packages according to your robot configuration.

Example:

```bash
ros2 launch navigation_pkg navigation.launch.py
```

## 👥 Development Team

| Team Member            | Primary Contributions                                                                                                                                                       |
| ---------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Nour Eldin** | Developed the computer vision pipeline, Robot GUI, and ROS2 software modules. Integrated perception components and contributed to the autonomous navigation implementation. |
| **Mark George**        | Developed the autonomous navigation system, robot control modules, ROS2 software components, and contributed to the overall system integration and architecture.            |
| **Mohamed Fayez**      | Developed the PID controller, IMU package, and motion control algorithms, including controller tuning and sensor integration.                                               |

---

## 📜 License

This project is intended for educational and research purposes.

---

## ⭐ Acknowledgments

This project was developed as part of an autonomous robotics initiative, integrating ROS2, embedded systems, computer vision, and autonomous navigation to build a complete mobile robotic platform.
