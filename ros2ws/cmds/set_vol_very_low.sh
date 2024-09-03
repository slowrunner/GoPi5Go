#!/bin/bash

# FILE: cmds/set_vol_very_low.sh

amixer -c 2 sset PCM 10%
~/GoPi5Go/ros2ws/cmds/say.sh 'Volume set very low 10 percent'
