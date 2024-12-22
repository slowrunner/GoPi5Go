#/bin/bash


echo -e "\n*** ECHO distance (TF) from the robot to the dock (apriltag 36h11 id 5)"
ros2 run tf2_ros tf2_echo base_link dock
