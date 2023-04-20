# Onboard Software

## Requirements

- Ubuntu Server 22.04.2
- Raspberry Pi 4
- Freenove Smartcar

## Installation

1. Update the system
   ```shell
   sudo apt-get update
   sudo apt-get upgrade
   ```

2. Install ROS by executing the following script.
    ```shell
    ./bin/install-ros.sh
    ```

3. Set up Raspberry Pi Camera
   ```shell
   sudo apt install raspi-config
   sudo raspi-config
   # enable Interface > Legacy Camera
   sudo reboot 
   ```
   
4. Install OpenCV
   ```shell
   sudo apt-get install python3-opencv
   ```

5. Confirm that it works
   ```shell
   python3 -c "import cv2; cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L); print(cap.read())"
   # it should print a Numpy array; otherwise Pi camera isn't properly setup
   ```

# Usage

Navigate to the ROS project root
```shell
cd ros_ws
```

Compile the ROS workspace
```shell
colcon build
source install/local_setup.bash
```

Run the camera node
```shell
ros2 run camera camera
```

In another shell, run the inference node
```shell
ros2 run inference inference
```

In another shell, run the ros2 bag recorder
```shell
ros2 bag record --all
```

