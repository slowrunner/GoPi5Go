#!/bin/bash

# FILE: cmds/set_vol_normal.sh

amixer -c 2 sset PCM 80%
~/GoPi5Go/ros2ws/cmds/say.sh 'Volume set to normal 80 percent'
