#!/bin/bash

amixer -D pulse sset Master 80%
~/GoPi5Go/ros2ws/cmds/say.sh 'Volume set to 80 percent' 
