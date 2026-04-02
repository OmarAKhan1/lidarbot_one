# LiDAR SLAM with ESP32 (micro-ROS + ROS2)

This repository enables you to generate maps using **SLAM** with a **LiDAR connected to an ESP32-S3**. The system communicates with a host computer via **micro-ROS**, and mapping is performed using the **slam_toolbox** package in ROS2.

---

## 🚀 Overview

* ESP32-S3 acts as the microcontroller interface for sensors and actuators
* LiDAR data is transmitted to a host machine using **micro-ROS**
* The host machine runs **ROS2** and performs SLAM using **slam_toolbox**
* Visualization and mapping are done in **RViz2**

---

## 🛠 Hardware Used

* ESP32-S3
* LD14P LiDAR
* STS3215 Servos (one for each wheel)
* Waveshare Bus Servo Adapter

---

## 💻 Required Software

Before running the project, make sure you have the following installed:

1. ROS2 *(tested using ROS2 Kilted Kaiju)*
2. micro-ROS
3. slam_toolbox
4. Arduino IDE (or equivalent, for uploading code to ESP32)

---

## ⚡ Setup Instructions

### 1. Upload Code to ESP32
Upload the following file to your ESP32-S3:

[esp32_SLAM.ino](./esp32_SLAM.ino)

Must install the following libraies for it to run successfully:
1. KAIA.ai library for LD14P LiDAR
2. FT&WS library for SCServo  

---

### 2. Run the System

After powering the ESP32-S3, open multiple terminals in your ROS2 workspace and run the following commands:

### Terminal 1: Start micro-ROS agent

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

### Terminal 2: Launch robot

ros2 launch lidarbot_one launch_robot.launch.py

### Terminal 3: Start RViz

rviz2

### Terminal 4: Start SLAM

ros2 launch lidarbot_one online_async_launch.py use_sim_time:=false

### Terminal 5: Teleoperation

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

---

## ✅ Expected Behavior

Once everything is running:

* You should see the robot model and LiDAR scan data in **RViz2**
* You can control the robot using your keyboard via teleop
* A map will begin generating in real time as the robot moves

---

## 🧪 Simulation Note

Some files in this repository are **not used for the physical robot**.

These are intended for running the robot in **Gazebo Sim** for testing and simulation purposes.

---

## 📌 Acknowledgments

* Based on Josh Newan's *articubot_one* project
* Built using ROS2, micro-ROS, and slam_toolbox

---

## 📬 Notes

* Make sure all devices are on the same network if required
* Double-check topic remappings if things don’t move or visualize correctly
* Ensure correct serial/UDP configuration for micro-ROS
