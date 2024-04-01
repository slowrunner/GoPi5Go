#!/bin/bash


while true; \
do echo -e "\n********** GoPi5Go  MONITOR ******************************"; \
echo -n `date +"%A %D"`; \
# echo -e '\n'; \
uptime; \
vcgencmd measure_temp && vcgencmd measure_clock arm && vcgencmd get_throttled; \
free -h; \
echo -e "\n"; \
pgrep -a safe
sleep 10; \
echo " "; \
done
