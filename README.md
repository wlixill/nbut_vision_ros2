# daheng_camera

A ROS2 packge for daheng USB3.0 industrial camera

## calibration usage

```
 sudo apt install ros-humble-camera-calibration-parsers
 
 sudo apt install ros-humble-camera-info-manager
 
 sudo apt install ros-humble-launch-testing-ament-cmake

colcon build 

source install/setup.bash

ros2 launch daheng_camera daheng_camera.launch.py

查看/image_raw 话题 发布频率

ros2 topic list  

ros2 topic hz /image_raw


启动相机标定节点

ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.019 image:=/image_raw camera:=/camera 

or

ros2 run camera_calibration cameracalibrator --size 7x11 --square 0.0361 image:=/image_raw camera:=/camera

8x6棋盘生成

<https://flowus.cn/lihanchen/share/ae25f784-4c72-456d-9dc1-e158e4270683>
```

标定板生成

<https://calib.io/pages/camera-calibration-pattern-generator>

## Params

- exposure_time
- gain
