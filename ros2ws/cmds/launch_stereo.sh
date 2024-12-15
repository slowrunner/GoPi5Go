#!/bin/bash

: '
/image/compressed
/image/compressedDepth
/image/theora
/left/camera_info
/left/image_rect
/left/image_rect/compressed
/left/image_rect/compressedDepth
/left/image_rect/theora
/right/camera_info
/right/image_rect
/right/image_rect/compressed
/right/image_rect/compressedDepth
/right/image_rect/theora
/stereo/camera_info
/stereo/converted_depth
/stereo/depth
/stereo/depth/compressed
/stereo/depth/compressedDepth
/stereo/depth/theora
/stereo/points

'

ros2 launch depthai_examples stereo.launch.py camera_model:=OAK-D-W use_rviz:=False
