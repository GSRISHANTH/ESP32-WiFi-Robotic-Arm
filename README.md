# 🤖 ESP32 Wi-Fi Robotic Arm for Industry 4.0

<p align="center">

<img src="images/dashboard.png" width="900">

</p>

<p align="center">

![ESP32](https://img.shields.io/badge/ESP32-Embedded-blue?style=for-the-badge)
![Embedded C](https://img.shields.io/badge/Embedded-C-blue?style=for-the-badge)
![Arduino IDE](https://img.shields.io/badge/Arduino-IDE-success?style=for-the-badge)
![Wi-Fi](https://img.shields.io/badge/Wi--Fi-HTTP-orange?style=for-the-badge)
![Industry 4.0](https://img.shields.io/badge/Industry-4.0-red?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Completed-brightgreen?style=for-the-badge)

</p>

---

# 📖 Overview

This project presents an **ESP32-based Wi-Fi Robotic Arm** designed for **Industry 4.0-inspired automation and smart manufacturing applications**.

The robotic arm is controlled through a **browser-based dashboard** hosted directly on the ESP32. Any device connected to the same Local Area Network (LAN) can control the robotic arm without requiring a dedicated mobile application.

The system supports:

- Real-time servo control
- Smooth non-blocking motion
- Trajectory recording
- Trajectory playback
- Emergency stop
- Resume operation
- Servo calibration
- Fail-safe recovery

The project demonstrates embedded firmware development, wireless communication, real-time robotic control, and industrial automation concepts using ESP32.

---

# 🎯 Objectives

- Develop a Wi-Fi controlled robotic arm using ESP32.
- Build an intuitive browser-based control dashboard.
- Enable smooth multi-servo movement.
- Record and replay robotic trajectories.
- Implement fail-safe recovery mechanisms.
- Demonstrate Industry 4.0 concepts using embedded systems.

---

# ✨ Key Features

- 🌐 Wi-Fi Based Control
- 📱 Browser-Based Dashboard
- ⚙ Real-Time Servo Control
- 🤖 Multi-Servo Coordination
- 🎯 Smooth Motion Algorithm
- 💾 Trajectory Recording
- ▶ Trajectory Playback
- 🛑 Emergency Stop
- 🔄 Resume Function
- 🛡 Fail-Safe Recovery
- 🔧 Servo Calibration

---

# 🛠 Hardware Components

| Component | Quantity |
|-----------|---------:|
| ESP32 Development Board | 1 |
| Servo Motors | 6 |
| Robotic Arm Kit | 1 |
| 18650 Li-ion Battery | 2 |
| Buck Converter | 1 |
| Battery Holder | 1 |
| Connecting Wires | As Required |

---

# 💻 Software Stack

- Arduino IDE
- Embedded C
- HTML
- CSS
- JavaScript
- ESP32 Wi-Fi Library
- ESP32Servo Library

---

# 🏗 System Architecture

```text
          Mobile / Laptop
                 │
            Wi-Fi Network
                 │
          ESP32 Web Server
                 │
       Motion Control Engine
                 │
        Servo Driver Control
                 │
         6-DOF Robotic Arm
```

---

# ⚙ Working Principle

1. ESP32 initializes as a Wi-Fi server.
2. Users access the control dashboard using a browser.
3. HTTP requests are transmitted to the ESP32.
4. ESP32 processes commands in real time.
5. Servo motors move smoothly using a non-blocking algorithm.
6. Motion sequences can be recorded.
7. Recorded trajectories can be replayed automatically.
8. Emergency stop and fail-safe mechanisms improve operational safety.

---

# 📂 Repository Structure

```text
ESP32-WiFi-Robotic-Arm
│
├── README.md
├── LICENSE
├── .gitignore
│
├── code
│   └── ESP32_WiFi_Robotic_Arm
│       └── ESP32_WiFi_Robotic_Arm.ino
│
├── docs
│   └── Presentation.pptx
│
├── images
│   ├── circuit.png
│   └── dashboard.png
│
└── videos
    ├── demo1.mp4
    └── demo2.mp4
```

---

# 📸 Circuit Diagram

<p align="center">
<img src="images/circuit.png" width="850">
</p>

---

# 🌐 Web Dashboard

<p align="center">
<img src="images/dashboard.png" width="900">
</p>

The dashboard enables:

- Individual Servo Control
- Real-Time Motion
- Trajectory Recording
- Automatic Replay
- Emergency Stop
- Resume Operation

---

# 🎥 Demonstration

The repository contains two demonstration videos:

- `videos/demo1.mp4`
- `videos/demo2.mp4`

These videos demonstrate:

- Wi-Fi based robotic control
- Smooth motion
- Dashboard operation
- Servo synchronization
- Pick-and-place movement

---

# 💻 Source Code

The firmware is located at:

```text
code/ESP32_WiFi_Robotic_Arm/ESP32_WiFi_Robotic_Arm.ino
```

Main modules include:

- Wi-Fi Web Server
- HTTP Communication
- Servo Driver
- Motion Controller
- Trajectory Recording
- Trajectory Playback
- Emergency Stop
- Fail-Safe Logic

---

# 📊 Results

The developed robotic system successfully demonstrated:

- Smooth multi-servo movement
- Low-latency Wi-Fi communication
- Stable browser-based control
- Accurate trajectory recording
- Reliable automatic playback
- Emergency stop functionality
- Robust fail-safe recovery
- Responsive embedded control

---

# 🚀 Future Enhancements

- Computer Vision
- OpenCV Integration
- ROS2 Support
- Inverse Kinematics
- AI-Based Motion Planning
- Edge AI
- Mobile Application
- Cloud Monitoring
- Voice Control

---

# 🏭 Applications

- Industry 4.0
- Smart Manufacturing
- Factory Automation
- Pick-and-Place Systems
- Embedded Robotics
- IoT Automation
- Educational Robotics
- Research Laboratories

---

# ▶️ Getting Started

## Clone Repository

```bash
git clone https://github.com/GSRISHANTH/ESP32-WiFi-Robotic-Arm.git
```

## Open Project

Open

```text
code/ESP32_WiFi_Robotic_Arm/ESP32_WiFi_Robotic_Arm.ino
```

using the Arduino IDE.

## Install Required Libraries

- ESP32 Board Package
- WiFi
- ESP32Servo
- Additional libraries used in the project

## Upload Firmware

- Select ESP32 Development Board
- Select COM Port
- Upload the firmware

## Access Dashboard

Open the IP address displayed on the Serial Monitor using any browser connected to the same Wi-Fi network.

---

# 📄 Documentation

The complete project presentation is available in:

```text
docs/Presentation.pptx
```

It includes:

- Project Objectives
- Hardware Details
- System Design
- Circuit Diagram
- Dashboard Design
- Experimental Results
- Future Scope

---

# 👨‍💻 Author

### G. Srishanth

**B.Tech – Electronics & Communication Engineering**

Amrita Vishwa Vidyapeetham, Bengaluru

📧 **Email:** gadesrishanth03@gmail.com

💼 **LinkedIn:**  
https://www.linkedin.com/in/srishanth-gade/

🐙 **GitHub:**  
https://github.com/GSRISHANTH

---

# ⭐ Show Your Support

If you found this project useful, consider giving this repository a ⭐.

Feedback and suggestions are always welcome.
