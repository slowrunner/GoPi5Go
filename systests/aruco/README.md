# ROS2 Aruco Markers

REF: https://github.com/fictionlab/ros_aruco_opencv

```
sudo apt install ros-humble-aruco-opencv ros-humble-aruco-opencv-msgs

cp /opt/ros/humble/share/aruco_opencv/config/aruco_tracker.yaml (/home/pi/GoPi5Go/ros2ws/) params/

$ pushd /opt/ros/humble/share/depthai_ros_driver/launch//

sudo cp camera.launch.py aruco_camera.launch.py
```

- Mods:  sudo nano aruco_camera.launch.py

```
# from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
...
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        parameters=[params_file, tf_params],
                        # Remap default '/oak/rgb' topic to '/camera' for aruco_opencv
                        remappings=[
                            ('/oak/rgb/image_raw', '/camera/image_raw'),
                            ('/oak/rgb/camera_info', '/camera/camera_info'),
                        ],
                    )
```  


Modify params/aruco_tracker.yaml for Dave's marker

Launch aruco_camera:

```
pi@GoPi5Go:DOCKER:~/GoPi5Go/ros2ws $ ros2 launch depthai_ros_driver aruco_camera.launch.py camera_model:=OAK-D-W params_file:=/home/pi/GoPi5Go/ros2ws/params/camera.yaml

```
Launch aruco tracker:
```
 ros2 launch aruco_opencv aruco_tracker.launch.xml

```


```
pi@GoPi5Go:DOCKER:~/GoPi5Go/ros2ws $ ros2 topic echo /aruco_detections 


header:
  stamp:
    sec: 1734286796
    nanosec: 425563519
  frame_id: oak_rgb_camera_optical_frame
markers:
- marker_id: 5
  pose:
    position:
      x: 0.05173255350509175
      y: -0.029103121047933316
      z: 0.5412244950190088
    orientation:
      x: 0.9975646146976297
      y: 0.026159965251779892
      z: -0.054218249093173264
      w: 0.035226086732218505
boards: []

```

### =====Alternate not requiring aruco_camera.launch.py to remap topics =====
REF: https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main

