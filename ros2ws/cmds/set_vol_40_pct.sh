#!/bin/bash

amixer -D pulse sset Master 40%
~/GoPi5Go/ros2ws/cmds/say.sh 'Volume set to 40 percent'
