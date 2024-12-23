#/bin/bash


echo -e "\n*** ECHO distance (TF) from the oak camera to the dock (apriltag 36h11 id 5)"
echo -e "ros2 run tf2_ros tf2_echo oak dock"

ros2 run tf2_ros tf2_echo oak dock
