#!/usr/bin/env python3
#
# This code is for power management on a Raspberry Pi 5 with GoPiGo3.
# (DI code for Pi3 and Pi4 is left in comments)
#
# GPIO 22 will be configured as input with pulldown. If pulled high, the RPi will shutdown immediately and halt.
#
# GPIO 23 needs to remain low impedance (output) set to a HIGH state. If GPIO 23 gets
# left floating (high impedance) the GoPiGo3 assumes the RPi has shut down fully.
# SW should never write GPIO 23 to LOW or set it as an INPUT.

# Use gpioinfo to view effect
# Before:
#	line  22:     "GPIO22"       unused   input  active-high 
# 	line  23:     "GPIO23"       unused   input  active-high 
# After:
#	line  22:     "GPIO22" "gopi5go_power.py" input active-high [used pull-down]
#	line  23:     "GPIO23" "gopi5go_power.py" output active-high [used]



# import RPi.GPIO as GPIO
import gpiod
import time
import os

# GPIO.setmode(GPIO.BCM)
chip = gpiod.Chip('gpiochip4')

# rpi_alive is a reference to GPIO23
rpi_alive = chip.get_line(23)

# GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# create reference to GPIO22 and set input with a pull down
gpg_pwr_button = chip.get_line(22)
gpg_pwr_button.request(consumer="gopi5go_power.py", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN)


# set GPIO23 to output
# GPIO.setup(23, GPIO.OUT)
rpi_alive.request(consumer="gopi5go_power.py", type=gpiod.LINE_REQ_DIR_OUT)

# GPIO.output(23, True)
rpi_alive.set_value(1)

while True:
    # if GPIO.input(22):
    if gpg_pwr_button.get_value():
        os.system("shutdown now -h")
    time.sleep(0.1)
