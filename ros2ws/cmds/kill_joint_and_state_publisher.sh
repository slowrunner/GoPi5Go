#!/bin/bash

echo -e "/n**** Killing joint_state and robot_state publishers"

killall joint_state_publisher
killall robot_state_publisher
