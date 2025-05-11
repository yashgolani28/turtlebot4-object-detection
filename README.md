# 🤖 TurtleBot4 Object Detection with ROS 2

A ROS 2-powered project that enables real-time object detection and localization using YOLOv8 and a TurtleBot4 robot. Ideal for autonomous navigation, exploration, and interaction tasks in indoor environments.

---

## 🧠 Features

- 📦 Real-time object detection using YOLOv8
- 🧭 ROS 2 integration with TurtleBot4
- 📍 Object position localization in map frame
- 🖼️ Live camera feed with bounding boxes
- 🗺️ Optional SLAM or Navigation2 integration

---

## 📷 Demo

> https://github.com/user-attachments/assets/52f1eab4-46b9-40da-8241-619d73a0cf95

---

## 🧰 Tech Stack

| Component    | Technology            |
|--------------|------------------------|
| Robot        | TurtleBot4              |
| Framework    | ROS 2 (Humble/Foxy)     |
| Perception   | YOLOv8, OpenCV, PyTorch |
| Visualization| RViz2                   |
| OS           | Ubuntu 22.04 + ROS 2    |

---

## 📁 Folder Structure

```

turtlebot4-object-detection/
├── src/                      # ROS 2 packages
│   └── object\_detection/     # Main node
├── launch/                   # Launch files
├── models/                   # YOLO weights
├── scripts/                  # Utility scripts
└── README.md

````

---
## Result

<p align="center">
  <img src="https://github.com/user-attachments/assets/d14f1b5e-63ad-46a5-ba92-83be6457ce5e" alt="Screenshot" width="450"/>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/c8c423a7-a9cb-43c3-b4f4-7f5e1986b41e" alt="WhatsApp Image" width="450"/>
</p>

---
## ⚙️ Installation

1. **Clone Repository**
   ```bash
   git clone https://github.com/yashgolani28/turtlebot4-object-detection.git
   cd turtlebot4-object-detection

2. **Install Dependencies**

   ```bash
   sudo apt update
   sudo apt install ros-humble-cv-bridge ros-humble-image-transport
   pip install ultralytics
   ```

3. **Download YOLOv8 Weights**

   ```bash
   cd models/
   wget https://path/to/yolov8n.pt
   ```

4. **Build the Workspace**

   ```bash
   colcon build
   source install/setup.bash
   ```

---

## 🚀 Usage

```bash
ros2 launch object_detection detection.launch.py
```

> Use `RViz2` to visualize detections or subscribe to `/detections` topic.

---

## 📦 Topics

* `/camera/image_raw` – raw video feed
* `/detections` – detected object class & bounding boxes
* `/tf` – for object localization

---

## 👨‍💻 Contributors

* Yash Golani
  
---

## 📬 Contact

For queries, contact via [LinkedIn](https://www.linkedin.com/in/yashgolani28) or open an issue on this repo.

```
