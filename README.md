# Interfacing USB Camera with ROS Noetic

## Project Overview

This repository contains a collection of ROS Noetic projects focused on interfacing a USB camera and utilizing various computer vision techniques. The projects include:

1. Interfacing Camera with ROS
2. Face Detection
3. Color Detection
4. Finger Count
5. Face Counting
6. Simulating Differential Robot with Color
7. Simulating Differential Robot with Finger Count

## Project Details

### 1. Interfacing Camera with ROS

This project demonstrates how to interface a USB camera with ROS Noetic. It includes a publisher node that captures video frames from the camera and publishes them as ROS messages.

- **Nodes**: `camera_publisher.py`, `camera_subscriber.py`
- **Topic**: `video_topic`

### 2. Face Detection

Using OpenCV's Haar Cascade Classifier, this project detects faces in the video feed from the USB camera and displays the detected faces with bounding boxes.

- **Nodes**: `face_pub.py`, `face_sub.py`
- **Topic**: `video_topic`

### 3. Color Detection

This project detects specific colors (red, yellow, blue) in the video feed. It demonstrates how to use HSV color space and thresholding for color detection.

- **Nodes**: `pub_color.py`, `sub_color.py`
- **Topic**: `video_topic`
- **Detected Colors**: Red, Yellow, Blue

### 4. Finger Count

Using the Mediapipe library, this project detects and counts the number of raised fingers in the video feed. It can be extended to recognize different hand gestures.

- **Nodes**: `hand_pub.py`, `hand_sub.py`
- **Topic**: `video_topic`

### 5. Face Counting

Building on the face detection project, this project counts the number of faces detected in the video feed and displays the count on the screen.

- **Nodes**: `face_c_pub.py`, `face_c_sub.py`
- **Topic**: `video_topic`

### 6. Simulating Differential Robot with Color

This project integrates color detection with robot simulation. The robot performs different actions based on the detected color in the video feed.

- **Nodes**: `diff_color_pub.py`, `diff_color_sub.py`
- **Topic**: `video_topic`
- **Actions**:
  - Red: Turn right
  - Yellow: Move forward
  - Blue: Turn left
  - No Color: Stop

### 7. Simulating Differential Robot with Finger Count

This project uses finger count detection to control a differential robot's movements. The robot responds to the number of raised fingers detected in the video feed.

- **Nodes**: `diff_hand_pub.py`, `diff_hand_sub.py`
- **Topic**: `video_topic`
- **Actions**:
  - 1 Finger: Move forward
  - 2 Fingers: Move backward or stop
  - 3 Fingers: Move left
  - 4 Fingers: Move right
  - No Fingers: Stop

## Installation and Usage

### Prerequisites

- ROS Noetic
- OpenCV
- Mediapipe
- USB Camera

### Installation

Clone this repository:

Install the required dependencies: 
```sh
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport
pip install opencv-python mediapipe
```
## **Running the Projects**
### **Start the ROS Master:**
```sh
roscore
```
Run the Publisher Node:
```sh
rosrun your_package_name camera_publisher.py
```
Run the Subscriber Nodes:
```sh
rosrun your_package_name face_detection_subscriber.py
```
## *note*
Don't forget to add the path in CMakeLists.txt:
```sh
catkin_install_python(PROGRAMS
  src/camera_subscriber.py
  src/camera_publisher.py
  src/face_pub.py
  src/face_sub.py
  src/pub_color.py
  src/sub_color.py
  src/hand_sub.py
  src/hand_pub.py
  src/face_c_pub.py
  src/face_c_sub.py
  src/diff_color_pub.py
  src/diff_color_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
If you are using a USB camera, add the port name on
```sh
videoCaptureObject = cv2.VideoCapture('/dev/video2')
```
or if it's the default camera, use
```sh
videoCaptureObject = cv2.VideoCapture(0).
```
## Simulating Differential Robot with Camera
### Step-by-Step Instructions
1.Start the ROS Master:
```sh
roslaunch master_description gazebo.launch
```
2.In another terminal, run the mapping launch file:
```sh
roslaunch master_description mymapping.launch
```
3.In another terminal, run the move base launch file:
```sh
roslaunch master_description move_base.launch
```
4.In a fourth terminal, run your publisher node:
```sh
rosrun script your_publisher_node.py
```
5.In a fifth terminal, run your subscriber node:
```sh
rosrun script your_subscriber_node.py
```
Replace your_publisher_node.py and your_subscriber_node.py with the actual filenames of your publisher and subscriber scripts.
