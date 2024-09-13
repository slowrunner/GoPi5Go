#!/bin/bash

# FILE:  nap_pi5_for.sh
# USAGE:  ./nap_pi5_for.sh [N.n hours]
# REQ: sudo -E rpi-eeprom-config --edit, add POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0

if [ "$#" -ne 1 ] ;
	then echo "Usage:  ./nap_pi5_for.sh NN.n (NN.n hours) "
	exit
fi
batt=`(/home/pi/GoPi5Go/plib/battery.py)`
echo "GoPi5Go-Dave is going to nap for $1 hours, vBatt: $batt"
~/GoPi5Go/utils/logMaintenance.py 'GoPi5Go-Dave is going to nap for '$1' hours, vBatt: $batt'

#espeak-ng -a 200 "I'm going to take a nap for "$1" hours"

# Speak at volume 200 ignore quietTime
~/GoPi5Go/plib/speak.py "Battery at ${batt} volts, I'm going to take a nap for ${1} hours" 200 True

# Convert hours to seconds to set alarm - "/ 1" converts to integer
napsecs=`(echo "scale=0; ($1 * 3600) / 1" | bc)`
naptime="+"$napsecs
# echo "naptime: "$naptime
echo -e "Issuing: echo "$naptime" | sudo tee /sys/class/rtc/rtc0/wakealarm"
echo $naptime | sudo tee /sys/class/rtc/rtc0/wakealarm
sudo shutdown -h +1

