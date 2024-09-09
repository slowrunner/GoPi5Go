#!/bin/bash


while true; \
do echo -e "\n********** ROS2 GoPi5Go Battery MONITOR ******************************"; \
echo -n `date +"%A %D"`; \
echo ""; \
uptime; \
if [ -f /usr/bin/docker ]; then
    vcgencmd measure_temp && vcgencmd measure_clock arm && vcgencmd get_throttled;
fi; \
free -h; \
if [ -f /usr/bin/docker ]; then
    python3 /home/pi/GoPi5Go/ros2ws/gopigo3_battery.py;
else
    ./check_battery.sh;
fi; \
sleep 300; \
echo " "; \
done
