#!/bin/bash

logfile="/home/pi/GoPi5Go/logs/odometer.log"

echo -e "\n*** TAIL ODOMETER.LOG ***"
tail $logfile

echo -e "\n*** TRAVEL FROM ODOMETRY.LOG ***"
totalMoved=`(awk -F'moved:' '{sum+=$2}END{printf "%.1f", sum;}' $logfile)`
totalMovedFt=`(echo "scale=1; ($totalMoved / 0.3048)" | bc)`
echo "Total Travel: " $totalMoved "meters" $totalMovedFt "feet"

