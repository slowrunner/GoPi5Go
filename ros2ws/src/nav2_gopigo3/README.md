# Nav2 GoPiGo3

This package is "lifted" from v1.0.4 of TurtleBot4 Navigation
(under Apache License) 

There are three modes of use:
- mapping (SLAM)
- localization within a map
- navigation within a map


Typical execution:

Run synchronous SLAM:
- ros2 launch nav2_gopigo3 slam.launch.py ['slam_params_file:=path/to/slam.yaml']
- start_gpgnav_slam.sh  (uncomment sync section)

Run asynchronous SLAM:
- ros2 launch nav2_gopigo3 slam.launch.py sync:=false ['slam_params_file:=path/to/slam.yaml'] 
- start_gpgnav_slam.sh  (uncomment async section)

Running localization with an existing map:
- ros2 launch nav2_gopigo3 localization.launch.py map:=/path/to/map.yaml ['slam_params_file:=path/to/localization.yaml']

Run navigation (Nav2):
- ros2 launch nav2_gopigo3 nav2.launch.py ['slam_params_file:=path/to/nav2.yaml']


