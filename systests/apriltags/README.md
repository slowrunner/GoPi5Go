# AprilTags For ROS 2 GoPi5Go-Dave Robot  

<img src="https://github.com/slowrunner/GoPi5Go/blob/main/systests/apriltags/Dave_Looking_At_AprilTag.jpg" width="400" />  


REF:  https://github.com/AprilRobotics/apriltag  

REF: april.eecs.umich.edu/pdfs/olson2011tags.pdf  
     - Edwin Olson, Univ. of Michigan, "AprilTag: A robust and flexible visual fiducial system"  


- For ArUco compatibility use tag36h11 family  

- Repo says "Most folks should use TagStandard41h12 family"  

- AprilTags are reported to produce more stable pose estimation and better recognition distance than ArUco markers.  
 
- FIRST robotics competitions use 36h11 April Tags  

<img src="https://github.com/slowrunner/GoPi5Go/blob/main/systests/apriltags/Dave_Looking_For_AprilTag_Marker.jpg" width="400" />  

### Printing April Tags  

```
mkdir systests/apriltags

- Note: wget tag files does not bring down good file - bring down the repo
git clone https://github.com/AprilRobotics/apriltag-imgs.git

cp apriltag-imgs/tagStandard41h12/tag41_12_00005.png .
cp apriltag-imgs/tag36h11/tag36_11_00005.png .
cp apriltag-imgs/tag_to_svg.py  .
chmod +x tag_to_svg.py

- make svg for printing
./tag_to_svg.py tag41_12_00005.png tag41_12_00005.svg --size=110
./tag_to_svg.py tag36_11_00005.png tag36_11_00005.svg --size=110

```

- Printing:  
  - open tag.svg file with Gimp  
  - Gimp will show a "Render Scalable Graphics" dialog, choose mm, enter 110 mm width  
  - File, Print - it really will print 110mm wide!  



# Setup AprilTag ROS2 Node (@christianrauch repo)  

- Note: Christian Rauch built the first ROS node for the GoPiGo3 platform in 2016  

```
cd ~/GoPi5Go/ros2ws/src  
git clone https://github.com/christianrauch/apriltag_ros.git  
git clone https://github.com/christianrauch/apriltag_msgs.git
rosdep install -i --from-path src  
  executes:  
    sudo -H apt-get install ros-humble-apriltag-msgs  
    sudo -H apt-get install ros-humble-apriltag  
colcon build --packages-select apriltag_ros apriltag_msgs

Add to dockerfile:

RUN sudo apt-get install -y \
    ros-humble-ament-cmake-clang-format \
    ros-humble-apriltag-msgs \
    ros-humble-apriltag 

```

### Initial Test with OAK-D-W stereo depth camera RGB image

```
cmds/launch_camera.sh

ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-W params_file:=/home/pi/GoPi5Go/ros2ws/params/camera.yaml 


cmds/launch_apriltags.sh

ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/oak/rgb/image_raw -r camera_info:=/oak/rgb/camera_info


pi@GoPi5Go:DOCKER:~/GoPi5Go/ros2ws $ ros2 topic echo --once /detections
header:
  stamp:
    sec: 1734577479
    nanosec: 245452683
  frame_id: oak_rgb_camera_optical_frame
detections:
- family: tag36h11
  id: 5
  hamming: 0
  goodness: 0.0
  decision_margin: 147.9541015625
  centre:
    x: 625.9994773351233
    y: 354.6748190548022
  corners:
  - x: 635.1885375976562
    y: 345.58746337890625
  - x: 617.0053100585936
    y: 344.8626403808593
  - x: 616.717529296875
    y: 363.8540344238281
  - x: 634.9660644531251
    y: 364.45690917968756
  homography:
  - -11.162475273847008
  - 1.2208475571718502
  - 625.9994773351233
  - -1.496145738725772
  - -8.845596787777708
  - 354.6748190548022
  - -0.0032822103560256953
  - 0.001746631286121959
  - 1.0
---

(dave_apriltag.yaml param file sets frame id for id 5 to "dock")

pi@GoPi5Go:DOCKER:~/GoPi5Go/ros2ws $ ros2 run tf2_ros tf2_echo oak dock
At time 1734834952.30679414
- Translation: [0.682, 0.046, -0.034]
- Rotation: in Quaternion [-0.493, -0.487, 0.489, 0.530]
- Rotation: in RPY (radian) [-1.532, -0.035, 1.523]
- Rotation: in RPY (degree) [-87.758, -1.980, 87.288]
- Matrix:
  0.047 -0.037 -0.998  0.682
  0.998  0.036  0.046  0.046
  0.035 -0.999  0.039 -0.034
  0.000  0.000  0.000  1.000
```  


