# 🤖 Autonomous Human-Following Mobile Robot

### Graduation Project | Mechatronics Engineering

An autonomous mobile robot designed to detect and follow a designated person in real time using AI vision and ultrasonic sensors.

The system combines embedded programming, computer vision, sensor integration, motor control, wireless communication, and autonomous navigation.

---

## 🎯 Project Overview

The robot is designed to autonomously follow a selected person while maintaining a safe distance.

The system uses:

- AI-based person tracking
- Ultrasonic obstacle/distance sensing
- Differential drive motion
- Real-time embedded control
- Wireless communication
- BLDC motor control

### Potential Applications

- Airports
- Hospitals
- Shopping malls
- Hotels
- Industrial environments
- Autonomous assistance systems

---

## 🏗️ System Architecture

```text
        ┌──────────────────┐
        │    HuskyLens     │
        │   AI Vision      │
        └────────┬─────────┘
                 │ I2C
                 ▼
        ┌──────────────────┐
        │     ESP32-S3     │
        │  Main Controller │
        └───────┬──────────┘
                │
       ┌────────┴─────────┐
       │                  │
       ▼                  ▼
┌──────────────┐   ┌──────────────┐
│  Ultrasonic  │   │  BLE / App   │
│   Sensors    │   │ Communication│
└──────────────┘   └──────────────┘
       │
       ▼
┌──────────────────────────┐
│     Motion Control       │
│      & Navigation        │
└────────────┬─────────────┘
             │
       ┌─────┴─────┐
       ▼           ▼
   Left Motor   Right Motor                                                                                                                                                                                                                   🔧 Hardware
Component	Purpose
ESP32-S3	Main real-time controller
HuskyLens	AI vision and target tracking
HC-SR04 ×5	Distance and obstacle detection
BLDC Motors ×2	Differential drive
BLDC Motor Controllers ×2	Motor control
PWM to 0–10V Modules ×2	Motor throttle interface
BLE	Wireless communication
💻 Software & Technologies
Programming
C
C++
Embedded C/C++
Embedded Systems
ESP32-S3
I2C
PWM
GPIO
BLE
Robotics
Differential Drive
Person Tracking
Obstacle Detection
Sensor Fusion
Real-Time Control
Tools
Arduino IDE
VS Code
Proteus
Git
GitHub
🎮 Control System

The robot determines its movement based on:

Target position detected by HuskyLens
Distance measurements from ultrasonic sensors
Target deviation from the robot's center
Safety distance
Motion control logic

The final implementation uses a P-controller for the robot's low-speed tracking behavior.

📡 Communication
HuskyLens → ESP32-S3

Communication is performed using I2C.

ESP32-S3 → Motor Controllers

Motor commands are generated using PWM signals.

Mobile Application → ESP32-S3

BLE is used for wireless control and monitoring.

📁 Repository Structure
Autonomous-Human-Following-Mobile-Robot/
│
├── Documents/
│   ├── Project_Report/
│   ├── Documentation/
│   └── Diagrams/
│
├── Images/
│   ├── Robot/
│   ├── Electronics/
│   └── Testing/
│
├── Source_Code/
│   └── Robot_Control/
│
├── Videos/
│   ├── Demonstration/
│   └── Testing/
│
└── README.md
📸 Project Photos
Complete Robot

Electronics

Testing

🎥 Demonstration

Project demonstration videos are available in the:

Videos

folder.

🧪 Testing

The system was tested for:

Person detection
Person following
Distance maintenance
Obstacle detection
Motor response
Direction control
System integration
Real-world movement
👨‍💻 My Responsibilities
Team Leader

My main responsibilities included:

System integration
Embedded software development
ESP32-S3 programming
HuskyLens integration
Ultrasonic sensor integration
Motor control
Sensor fusion
Testing and debugging
Troubleshooting hardware/software issues
🏆 Achievement

Best Graduation Project – Academic Year 2025/2026

Egyptian Academy for Engineering and Advanced Technology (EAE&AT)

Project:

Autonomous Human-Following Mobile Robot

🚀 Future Improvements

Possible future developments include:

Improved person recognition
SLAM and autonomous mapping
LiDAR integration
Advanced obstacle avoidance
ROS/ROS 2 integration
Improved trajectory control
GPS/indoor positioning
AI-based navigation
🛠️ Technologies Summary

ESP32-S3 C++ Embedded C HuskyLens I2C PWM BLE

Ultrasonic Sensors BLDC Motors Differential Drive

Robotics Embedded Systems Computer Vision Automation
