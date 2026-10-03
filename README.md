
# 🤖 Autonomous Human-Following Mobile Robot

### Graduation Project | Mechatronics Engineering | 2025–2026

An autonomous mobile robot designed to detect and follow a designated person in real time using AI vision and ultrasonic sensors.

The system combines embedded programming, computer vision, sensor integration, motor control, wireless communication, and autonomous navigation.

---

## 🎯 Project Overview

The Autonomous Human-Following Mobile Robot is designed to autonomously detect and follow a selected person while maintaining a safe distance.

The robot uses an AI vision camera to track the target person and multiple ultrasonic sensors to monitor the surrounding environment and improve safety during movement.

The project integrates hardware and software components into a real-time autonomous robotic platform.

### Potential Applications

- Airports
- Hospitals
- Shopping Malls
- Hotels
- Industrial Environments
- Autonomous Assistance Systems

---

## 🏗️ System Architecture

```text
                    ┌──────────────────┐
                    │    HuskyLens     │
                    │   AI Vision      │
                    │ Person Tracking  │
                    └────────┬─────────┘
                             │
                            I2C
                             │
                             ▼
                    ┌──────────────────┐
                    │     ESP32-S3     │
                    │ Main Controller  │
                    └────────┬─────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
              ▼              ▼              ▼
      ┌──────────────┐ ┌─────────────┐ ┌──────────────┐
      │  Ultrasonic  │ │     BLE     │ │    Motion    │
      │   Sensors    │ │ Application │ │   Control    │
      └──────────────┘ └─────────────┘ └──────┬───────┘
                                              │
                                    ┌─────────┴─────────┐
                                    │                   │
                                    ▼                   ▼
                              Left BLDC Motor    Right BLDC Motor
````

---

## 🔧 Hardware

| Component                 | Purpose                                     |
| ------------------------- | ------------------------------------------- |
| ESP32-S3                  | Main real-time controller                   |
| HuskyLens                 | AI vision and target tracking               |
| HC-SR04 ×5                | Distance measurement and obstacle detection |
| BLDC Motors ×2            | Differential drive                          |
| BLDC Motor Controllers ×2 | Motor control                               |
| PWM to 0–10V Modules ×2   | Motor throttle interface                    |
| BLE                       | Wireless communication                      |

---

## 💻 Software & Technologies

### Programming

* C
* C++
* Embedded C/C++

### Embedded Systems

* ESP32-S3
* GPIO
* I2C
* PWM
* BLE
* Real-Time Control

### Robotics

* Human/Person Tracking
* Differential Drive
* Obstacle Detection
* Sensor Fusion
* Motion Control
* Autonomous Navigation

### Tools

* Arduino IDE
* VS Code
* Proteus
* MATLAB / Simulink
* Git
* GitHub

---

## 🎮 Control System

The robot determines its movement using information collected from the vision and ultrasonic sensing systems.

The control logic considers:

1. Target position detected by HuskyLens
2. Target deviation from the robot center
3. Distance between the robot and the target
4. Ultrasonic sensor measurements
5. Safety distance
6. Required movement direction and speed

The final implementation uses a **P-Controller** for the robot's low-speed human-following behavior.

---

## 📡 Communication

### HuskyLens → ESP32-S3

The HuskyLens AI vision camera communicates with the ESP32-S3 using **I2C**.

The camera provides information about the detected target, including its position in the camera frame.

### ESP32-S3 → Motor Controllers

The ESP32-S3 generates **PWM control signals** that are converted to the required throttle signal for the BLDC motor controllers.

### Mobile Application → ESP32-S3

**Bluetooth Low Energy (BLE)** is used for wireless communication and manual control/monitoring.

---

## 📐 Robot Movement

The robot uses a **differential drive system**.

The two rear BLDC motors independently control the left and right sides of the robot.

The movement direction is determined by the target's position relative to the center of the HuskyLens camera frame.

```text
                FRONT
                  ↑
        ┌───────────────────┐
        │    HuskyLens      │
        │    AI Camera      │
        └───────────────────┘
                  │
                  │
          Target Person
                  │
                  ▼
        ┌───────────────────┐
        │                   │
        │      ROBOT        │
        │                   │
        │                   │
        │  BLDC         BLDC│
        │  LEFT         RIGHT│
        └───────────────────┘
```

---

## 📁 Repository Structure

```text
Autonomous-Human-Following-Mobile-Robot/
│
├── Documents/
│
├── Images/
│
├── Source_Code/
│
├── Videos/
│
├── README.md
│
└── LICENSE
```

---

## 📸 Project Photos

Project photos, hardware images, electronics, and testing photos are available in the:

**[Images](Images/)**

folder.

---

## 🎥 Demonstration Videos

Project demonstration and testing videos are available in the:

**[Videos](Videos/)**

folder.

---

## 🧪 Testing

The robot was tested in real-world environments to evaluate:

* Target person detection
* Human following
* Distance maintenance
* Direction control
* Obstacle detection
* Ultrasonic sensing
* Motor response
* Communication
* System integration
* Hardware/software debugging

The testing process included both individual subsystem testing and complete system integration.

---

## 👨‍💻 My Responsibilities

### Team Leader

As the Team Leader, my main responsibilities included:

* System integration
* Embedded software development
* ESP32-S3 programming
* HuskyLens AI vision integration
* Ultrasonic sensor integration
* Motor control
* Sensor fusion
* Real-time control logic
* Hardware/software debugging
* System testing
* Troubleshooting

---

## 🏆 Achievement

### Best Graduation Project – Academic Year 2025/2026

**Egyptian Academy for Engineering and Advanced Technology (EAE&AT)**

The project was awarded **Best Graduation Project** for the academic year 2025/2026.

**Project:**

> Autonomous Human-Following Mobile Robot

---

## 🚀 Future Improvements

Possible future developments include:

* Advanced person recognition
* Improved obstacle avoidance
* LiDAR integration
* SLAM and autonomous mapping
* ROS / ROS 2 integration
* Advanced trajectory control
* AI-based navigation
* Indoor positioning
* Improved safety and navigation algorithms
* Autonomous path planning

---

## 🛠️ Technologies Summary

`ESP32-S3`
`C++`
`Embedded C`
`HuskyLens`
`I2C`
`PWM`
`BLE`
`HC-SR04`
`BLDC Motors`
`Differential Drive`
`Sensor Fusion`
`Computer Vision`
`Robotics`
`Embedded Systems`
`Real-Time Control`

---

## 📄 Project Documentation

Additional project documentation, technical files, diagrams, and reports can be found in the:

**[Documents](Documents/)**

folder.

---

## 👥 Project Team

This project was developed as a graduation project by a multidisciplinary Mechatronics Engineering team.

### Team Leader

**Sabry Mohamed Sabry**

Mechatronics Engineer | Embedded Systems | Robotics | Automation

---

## 📫 Contact

### LinkedIn

[Sabry Mohamed Sabry](https://www.linkedin.com/in/sabry-mohamed-94b395286/)

### GitHub

[SabryMohamedSabry13](https://github.com/SabryMohamedSabry13)

---

## ⭐ Project

If you find this project interesting, feel free to explore the source code, documentation, images, and demonstration videos.

**Built with Embedded Systems, Robotics, AI Vision, and Mechatronics Engineering.**

---

