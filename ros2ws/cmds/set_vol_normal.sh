#!/bin/bash

# FILE: cmds/set_vol_normal.sh

amixer -c 2 sset PCM 80%
~/wali_pi5/c3ws/cmds/say.sh 'Volume set to normal 80 percent'
