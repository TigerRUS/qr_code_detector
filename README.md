# QR codes processing

## Packages Overview

### 1. qr_detector
This node:
- Subscribes to a camera image topic
- Processes the images to detect QR codes using ZBar library
- Estimates the QR code position relative to the camera
- Publishes:
  - The decoded QR code ID to `/qr_detector/qr_id` topic
  - The processed image with visualizations to `/qr_decode` topic

### 2. qr_listener
This node:
- Subscribes to `/qr_detector/qr_id` topic
- When a new QR code is detected:
  - Gets the robot's current position in the map frame using TF
  - Logs the QR ID and robot position to a file
  - Triggers a buzzer (publishes to `/Buzzer` topic)
  - Ensures each QR code is only logged once

## Dependencies

- ROS1
- OpenCV
- ZBar library
- ROS packages:
  - `roscpp`
  - `sensor_msgs`
  - `cv_bridge`
  - `tf`
  - `image_transport`
  - `std_msgs`
  - `geometry_msgs`

## Installation

1. Clone the packages to your catkin workspace `src` folder:
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/TigerRUS/qr_code_detector.git
   ```

   Install dependencies:
   ```
   sudo apt-get install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-image-transport
   sudo apt-get install libzbar-dev
   ```

   Build the packages:
   ```
   cd ~/catkin_ws
   catkin_make --only-pkg-with-deps qr_detector
   catkin_make --only-pkg-with-deps qr_listener
   source devel/setup.bash
   ```

## Configuration

1. qr_detector
   
Modify launch/qr_detector.launch to set:

* Camera topic (default: /camera/rgb/image_raw)

* Camera info topic (default: /camera/rgb/camera_info)

* QR code physical size (default: 0.04)

2. qr_listener

Modify launch/qr_listener.launch to set:

* Output file path (default: /root/ros_files/qr_coordinates.txt)

* QR ID topic (default: /qr_detector/qr_id)

## Usage

First, bring up your robot's systems:

```
roslaunch [your_robot_package] bringup.launch
```

This should start:

- Camera driver

- LiDAR

- Motion system

- TF transforms

Run the QR detection system:

```
roslaunch qr_detector qr_detector.launch
roslaunch qr_listener qr_listener.launch
```

To view the processed images with rqt:
```
rqt_image_view
```

## Output Format

The QR codes and positions are logged in the format: QR_ID:X Y Yaw

Where:

QR_ID is the decoded QR code content

X, Y are the robot's position coordinates

Yaw is the robot's orientation

## Notes

The system requires proper TF transforms between map and base_link frames

Each QR code will only be logged once per session

The buzzer will sound briefly when a new QR code is detected

For best results, ensure good lighting conditions for QR code detection
