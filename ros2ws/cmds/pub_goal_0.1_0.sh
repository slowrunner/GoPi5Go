#!/bin/bash

# FILE:  pub_goal_0.1_0.sh

# PURPOSE: Issues a goal to the Nav2 node to move the robot to {0.100,0} 

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Publish goal {0.1,0} (x,y)"
echo -e "ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \"{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.100, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\""
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.1, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

