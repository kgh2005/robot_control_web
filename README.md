# RO:BIT Web Interface for Robot Visualization & Control

This project provides a web-based interface for visualizing and controlling a ROS 2 robot. It is designed to work with **ROS 2 Humble** on **Ubuntu 22.04**, and communicates via the `rosbridge_websocket` interface.

---

## ✅ Requirements

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **rosbridge_server**
- **roslibjs (JavaScript library for ROS)**
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
```

## YOLO Detection Result Image Publisher (ROS2)
```bash
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

// 퍼블리셔 선언 (class 안에서)
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

// 노드 생성자 등에서 퍼블리셔 초기화
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/yolo/image", 10);

// YOLO 결과 그린 후 이미지 전송
auto output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_image).toImageMsg();
output_msg->header.stamp = rclcpp::Clock().now();
image_pub_->publish(*output_msg);
```