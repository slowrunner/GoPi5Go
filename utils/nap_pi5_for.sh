#!/bin/bash

# FILE:  nap_pi5_for.sh
# USAGE:  ./nap_pi5_for.sh [N.n hours]
# REQ: sudo -E rpi-eeprom-config --edit, add POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0

if [ "$#" -ne 1 ] ;
	then echo "Usage:  ./nap_pi5_for.sh NN.n (NN.n hours) "
	exit
fi
echo "WaLiPi5 is going to nap for $1 hours"
~/GoPi5Go/utils/logMaintenance.py 'WaLiPi5 is going to nap for '$1' hours'
# ~/GoPi5Go/utils/say.sh "I'm going to take a nap for "$1" hours"

# Convert hours to seconds to set alarm - "/ 1" converts to integer
napsecs=`(echo "scale=0; ($1 * 3600) / 1" | bc)`
naptime="+"$napsecs
# echo "naptime: "$naptime
echo -e "Issuing: echo "$naptime" | sudo tee /sys/class/rtc/rtc0/wakealarm"
echo $naptime | sudo tee /sys/class/rtc/rtc0/wakealarm
sudo shutdown -h +1

