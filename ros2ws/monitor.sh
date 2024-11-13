#!/bin/bash


while true; \
do echo -e "\n********** ROS2 GoPiGo3 MONITOR ******************************"; \
echo -n `date +"%A %D"`; \
echo ""; \
uptime; \
if [ -f /usr/bin/docker ]; then
    vcgencmd measure_temp && vcgencmd measure_clock arm && vcgencmd get_throttled; \
    python3 /home/pi/GoPi5Go/plib/battery.py; 
fi; \
free -h; \
python3 /home/pi/GoPi5Go/ros2ws/gopigo3_battery.py; \
sleep 10; \
echo " "; \
done
