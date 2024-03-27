#!/bin/bash

echo -e "Last filesystem check:"
sudo tune2fs -l /dev/mmcblk0p2 | grep "Last checked"

echo -e "Configuring to run e2fsk next boot"
echo -e "and every 5 boots"
echo -e "and every 7 days"
echo -e "and remount read-only if errors are found"
echo -e "\n"
sudo tune2fs -c 5 -C 6 -i 7d -e remount-ro /dev/mmcblk0p2
