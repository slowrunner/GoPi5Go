#!/bin/bash

# Reference: https://itsfoss.com/disable-ipv6-ubuntu-linux/
# Don't forget to sudo sysctl -p

cd /home/pi/GoPi5Go/config
cp /etc/sysctl.conf etc_sysctl.conf.orig
sudo cp  etc_sysctl.conf.ipv6_disabled /etc/sysctl.conf
cp /etc/sysctl.conf etc_sysctl.conf.installed
