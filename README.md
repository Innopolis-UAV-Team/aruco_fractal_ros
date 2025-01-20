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
### 1. Build OpenCV with contrib
You can build OpenCV with contrib modules using the provided script `install_opencv.sh`:
```sh
cd <path_to_your_workspace>/aruco_fractal_ros
chmod +x install_opencv.sh
./install_opencv.sh
```

### 2. Build aruco
Download the latest version from: https://sourceforge.net/projects/aruco/files/

### 3. Clone the repository into your catkin workspace
```sh
cd ~/catkin_ws/src
git clone <repository_url>
```

### 4. Install the required dependencies
```sh
sudo apt-get install ros-<distro>-aruco-msgs ros-<distro>-cv-bridge ros-<distro>-image-transport ros-<distro>-message-filters ros-<distro>-roscpp ros-<distro>-rospy ros-<distro>-std-srvs ros-<distro>-tf2 ros-<distro>-tf2-geometry-msgs ros-<distro>-tf2-ros
sudo apt-get install libopencv-dev
sudo apt-get install ros-<distro>-aruco
```

### 5. Build the package
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
- `camera_info_topic_name`: The topic name for the camera info.
- `marker_topic_name`: The topic name for the detected marker.
- `rate`: The rate at which the node processes images.
- `window_size`: The size of the window for averaging marker positions.
- `marker_size`: The size of the marker.
- `parent_frame`: The parent frame for the marker.
- `marker_frame`: The frame for the marker.
- `marker_config`: The configuration for the fractal marker.
- `display_image`: Whether to display the processed image.
- `invert_image`: Whether to invert the image.
- `calibration_path`: The path to the camera calibration file.

## Nodes
### aruco_fractal_ros_node
This node subscribes to the image and camera info topics, detects fractal markers, and publishes the marker poses.

#### Subscribed Topics
- `image_topic_name` (`sensor_msgs/Image`): The input image topic.
- `camera_info_topic_name` (`sensor_msgs/CameraInfo`): The camera info topic.

#### Published Topics
- `marker_topic_name` (`geometry_msgs/PoseStamped`): The detected marker pose.
- `/aruco/image` (`sensor_msgs/Image`): The processed image with detected markers.

## Files
- `aruco_fractal_ros_node.cpp`: The main node implementation.
- `aruco_fractal.cpp`: The implementation of the ArUco fractal detection.
- `aruco_fractal.h`: The header file for the ArUco fractal detection.
- `test_node.cpp`: A test node for detecting fractal markers in a video or live stream.
- `fractal.launch`: The launch file for starting the node.
- `d435_720p.yaml`: test Camera calibration file for D435 camera.
- `fisheye1.yaml`: test Camera calibration file for fisheye camera 1.
- `fisheye2.yaml`: test Camera calibration file for fisheye camera 2.
- `CMakeLists.txt`: The CMake build configuration.
- `package.xml`: The package configuration file.

## Authors
- Innopolis-UAV-Team
- devittdv@gmail.com