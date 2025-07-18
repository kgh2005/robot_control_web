# RO:BIT Web Interface for Robot Visualization & Control

This project provides a **web-based interface** for interacting with multiple ROS 2 robots.  
It uses the `rosbridge_websocket` interface for communication and is designed with scalability and extensibility in mind.

> ✅ Designed for **ROS 2 Humble** on **Ubuntu 22.04**

---

## 🔗 GitHub Repository

➡️ **[https://github.com/kgh2005/web_control_bridge.git](https://github.com/kgh2005/web_control_bridge.git)**

This repository contains the full ROS 2 package and web interface source code for enabling web-based control and visualization across multiple robots in real-time.

---

## ✅ Requirements

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **rosbridge_server**
- **roslibjs** (JavaScript library for ROS)
- **web_video_server**
- **VSCode Live Server** (or any HTTP server)

---

## 🔧 Installation & How to Run

```bash
# 1. Install rosbridge server
sudo apt install ros-humble-rosbridge-server

# 2. Install and run web_video_server for image streaming
sudo apt install ros-${ROS_DISTRO}-web-video-server

# (Optional) If not available via apt, clone and build manually:
# git clone https://github.com/RobotWebTools/web_video_server.git

# 3. Launch rosbridge websocket server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 4. Launch the web video server
ros2 run web_video_server web_video_server

# 5. Open the web interface
# (A) Using VSCode Live Server:
# - Open this project folder in VSCode
# - Open index.html
# - Right-click and select "Open with Live Server"
