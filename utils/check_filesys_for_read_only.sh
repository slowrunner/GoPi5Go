#!/bin/bash

echo -e "Root file system status: (ro: read-only,  rw: read-write)"
more /proc/mounts | grep /dev/mmcblk0p2
