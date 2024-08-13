#!/bin/bash

batt=`(/home/pi/GoPi5Go/plib/battery.py)`
/home/pi/GoPi5Go/utils/logMaintenance.py "Routine Reboot - $batt"
sudo reboot
