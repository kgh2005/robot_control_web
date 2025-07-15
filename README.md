# RO:BIT Web Interface for Robot Visualization & Control

This project provides a web-based interface for visualizing and controlling a ROS 2 robot. It is designed to work with **ROS 2 Humble** on **Ubuntu 22.04**, and communicates via the `rosbridge_websocket` interface.

---

## ✅ Requirements

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **rosbridge_server**
- **roslibjs (JavaScript library for ROS)**
- **VSCode Live Server** (or any HTTP server)

---

## 🔧 Installation & How to Run

```bash
# 1. Install rosbridge_server
sudo apt install ros-humble-rosbridge-server

# 2. Launch rosbridge WebSocket server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 3. Launch the Web Interface

# (A) Using VSCode Live Server
# - Open this project folder in VSCode
# - Open index.html
# - Right-click and select "Open with Live Server"

