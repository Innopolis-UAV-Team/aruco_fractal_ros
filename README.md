# aruco_fractal_ros

## Overview
`aruco_fractal_ros` is a ROS package designed to detect and track ArUco fractal markers using OpenCV and ROS. The package subscribes to image topics, processes the images to detect fractal markers, and publishes the detected marker poses.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Dependencies
This package depends on the following ROS packages and system libraries:
- `aruco_msgs`
- `cv_bridge`
- `image_transport`
- `message_filters`
- `roscpp`
- `rospy`
- `std_srvs`
- `tf2`
- `tf2_geometry_msgs`
- `tf2_ros`
- `aruco`
- `OpenCV`

## Installation
1. Build aruco

download last version
https://sourceforge.net/projects/aruco/files/

2. Clone the repository into your catkin workspace:
    ```sh
    cd ~/catkin_ws/src
    git clone <repository_url>
    ```
3. Install the required dependencies:
    ```sh
    sudo apt-get install ros-<distro>-aruco-msgs ros-<distro>-cv-bridge ros-<distro>-image-transport ros-<distro>-message-filters ros-<distro>-roscpp ros-<distro>-rospy ros-<distro>-std-srvs ros-<distro>-tf2 ros-<distro>-tf2-geometry-msgs ros-<distro>-tf2-ros
    sudo apt-get install libopencv-dev
    sudo apt-get install ros-<distro>-aruco
    ```
4. Build the package:
    ```sh
    cd ~/catkin_ws
    catkin build
    ```

## Usage
1. Source your workspace:
    ```sh
    source ~/catkin_ws/devel/setup.bash
    ```
2. Launch the node using the provided launch file:
    ```sh
    roslaunch aruco_fractal_ros fractal.launch
    ```

## Configuration
The package can be configured using parameters specified in the launch file or via the parameter server. The following parameters are available:
- `image_topic_name`: The topic name for the input image.
- [camera_info_topic_name](http://_vscodecontentref_/1): The topic name for the camera info.
- `marker_topic_name`: The topic name for the detected marker.
- [rate](http://_vscodecontentref_/2): The rate at which the node processes images.
- [window_size](http://_vscodecontentref_/3): The size of the window for averaging marker positions.
- [marker_size](http://_vscodecontentref_/4): The size of the marker.
- [parent_frame](http://_vscodecontentref_/5): The parent frame for the marker.
- [marker_frame](http://_vscodecontentref_/6): The frame for the marker.
- [marker_config](http://_vscodecontentref_/7): The configuration for the fractal marker.
- [display_image](http://_vscodecontentref_/8): Whether to display the processed image.
- [invert_image](http://_vscodecontentref_/9): Whether to invert the image.
- [calibration_path](http://_vscodecontentref_/10): The path to the camera calibration file.

## Nodes
### aruco_fractal_ros_node
This node subscribes to the image and camera info topics, detects fractal markers, and publishes the marker poses.

#### Subscribed Topics
- `image_topic_name` (`sensor_msgs/Image`): The input image topic.
- [camera_info_topic_name](http://_vscodecontentref_/11) (`sensor_msgs/CameraInfo`): The camera info topic.

#### Published Topics
- `marker_topic_name` (`geometry_msgs/PoseStamped`): The detected marker pose.
- `/aruco/image` (`sensor_msgs/Image`): The processed image with detected markers.

## Files
- [aruco_fractal_ros_node.cpp](http://_vscodecontentref_/12): The main node implementation.
- [aruco_fractal.cpp](http://_vscodecontentref_/13): The implementation of the ArUco fractal detection.
- [aruco_fractal.h](http://_vscodecontentref_/14): The header file for the ArUco fractal detection.
- [test_node.cpp](http://_vscodecontentref_/15): A test node for detecting fractal markers in a video or live stream.
- [fractal.launch](http://_vscodecontentref_/16): The launch file for starting the node.
- [d435_720p.yaml](http://_vscodecontentref_/17): Camera calibration file for D435 camera.
- [fisheye1.yaml](http://_vscodecontentref_/18): Camera calibration file for fisheye camera 1.
- [fisheye2.yaml](http://_vscodecontentref_/19): Camera calibration file for fisheye camera 2.
- [CMakeLists.txt](http://_vscodecontentref_/20): The CMake build configuration.
- [package.xml](http://_vscodecontentref_/21): The package configuration file.

## Authors
- Innopolis-UAV-Team

## Acknowledgments
- This package uses the ArUco library for marker detection.
- Special thanks to the ROS community for their support and contributions.