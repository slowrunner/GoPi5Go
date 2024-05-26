#!/bin/bash

# FILE: cmds/set_vol_very_low.sh

amixer -c 2 sset PCM 10%
~/wali_pi5/c3ws/cmds/say.sh 'Volume set very low 10 percent'
