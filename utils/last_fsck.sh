#!/bin/bash

echo -e "Filesystem: /dev/mmcblk0p2 (/)"
sudo tune2fs -l /dev/mmcblk0p2 | grep "Last checked"

