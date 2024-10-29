# ROS2 Lidar /scan Topic Client in Python

Writes cardinal point distances from /scan topic to console

## Launch

To run: `ros2 run scan_client  scan_client`


```
Left (-0): XX.xxx   Forward (0): XX.xxx  Backward (180): XX.xxx  Right (-90): XX.xxx

or in Debug mode:

************* DEBUG 244 **********
*** Entering Scan Client Callback
*** angle_min: -180 max: 180 increment: 0.65

*** range[555] index - left 416 front 276 back 554 right 138

left: 0.264 cnt: 2170 non-zero: 98.9

front: 0.492 cnt: 2194 non-zero: 100.0

back: 0.201 cnt: 2194 non-zero: 100.0

right: 0.469 cnt: 2193 non-zero: 100.0

scan_msg.ranges[274]: 0.502 always 0? no
scan_msg.ranges[275]: 0.497 always 0? no
scan_msg.ranges[276]: 0.492 always 0? no
scan_msg.ranges[277]: 0.487 always 0? no
scan_msg.ranges[278]: 0.482 always 0? no
************* DEBUG **********

left: 0.264 front: 0.492 back: 0.201 right: 0.468

```

NOTE:  

Use cardboard playground to ensure good returns.  
(Dave on dock will lose a lot of ranges on black sim machine, and black UPS)  

Best ydlidar.yaml:  

```
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: base_scan
    ignore_array: ""
    baudrate: 128000
    lidar_type: 1
    device_type: 6
    sample_rate: 5            <<-- 5kHz sampling
    abnormal_check_count: 4
    fixed_resolution: true
    reversion: false
    inverted: true
    auto_reconnect: true
    isSingleChannel: false
    intensity: false
    support_motor_dtr: true
    angle_max: 180.0
    angle_min: -180.0
    range_max: 10.0
    range_min: 0.12
    frequency: 9.0            <<-- 9 rev/second (9Hz) will actually publish about 8.9 Hz
    invalid_range_is_inf: false

```

```
pi@GoPi5Go:DOCKER:~/GoPi5Go/ros2ws $ ros2 topic hz /scan
WARNING: topic [/scan] does not appear to be published yet
average rate: 8.957
	min: 0.105s max: 0.119s std dev: 0.00362s window: 10
average rate: 8.907
	min: 0.105s max: 0.119s std dev: 0.00410s window: 19
average rate: 8.927
	min: 0.105s max: 0.119s std dev: 0.00442s window: 28
average rate: 8.920
	min: 0.103s max: 0.119s std dev: 0.00454s window: 37

```
