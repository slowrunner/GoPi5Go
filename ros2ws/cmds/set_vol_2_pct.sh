#!/bin/bash

amixer -D pulse sset Master 2%
~/wali_pi5/c3ws/cmds/say.sh 'Volume set to 2 percent'
