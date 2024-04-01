#!/bin/bash


# pi@GoPi5Go:~/GoPi5Go/systests/gpg_power_service $ python3 gopigo3_power.py 
# Traceback (most recent call last):
#   File "/home/pi/GoPi5Go/systests/gpg_power_service/gopigo3_power.py", line 23, in <module>
#     GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# RuntimeError: Cannot determine SOC peripheral base address

echo -e "Removing non-working RPi.GPIO supplied with Bookworm"
sudo apt remove python3-rpi.gpio
echo-e "Installing working RPi.GPIO "
sudo pip3 install rpi-lgpio --break-system-packages
