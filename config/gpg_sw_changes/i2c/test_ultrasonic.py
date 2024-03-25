#!/usr/bin/env python
# Dexter Industries Distance Sensor example for the GoPiGo3
#
# This example shows a basic example to read sensor data from the Dexter Industries Distance Sensor.  This sensor is a white PCB.
#
# Connect the Dexter Industries Distance Sensor to an I2C port on the GoPiGo3.
# You can find the Distance Sensor here: https://www.dexterindustries.com/shop/distance-sensor/
# Have a question about this example?  Ask on the forums here:
# http://forum.dexterindustries.com/c/gopigo
#
# Initial Date: 16 Jun 2017  John Cole
# http://www.dexterindustries.com/GoPiGo
'''
## License
 Copyright (C) 2017  Dexter Industries

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
'''

# import the GoPiGo3 drivers
import time
import easygopigo3 as easy

# This example shows how to read values from the Distance Sensor

# Create an instance of the GoPiGo3 class.
# GPG will be the GoPiGo3 object.
gpg = easy.EasyGoPiGo3()

print("EasyGoPiGo3 obj created")

# Create an instance of the Ultrasonic Sensor class.
# I2C1 and I2C2 are just labels used for identifyng the port on the GoPiGo3 board.
# But technically, I2C1 and I2C2 are the same thing, so we don't have to pass any port to the constructor.
my_ultrasonic_sensor = gpg.init_ultrasonic_sensor()

if my_ultrasonic_sensor:
  print("ultrasonic sensor created")
  print("dist:",my_ultrasonic_sensor.read_range_single(),"mm")
else:
  print("distance sensor not created, exiting")

if my_easy_distance_sensor:
  print("easy distance sensor created")
  print("dist:",my_easy_distance_sensor.read_range_single(),"mm")

else:
  print("easy distance sensor not created, exiting")
  exit()

while True:
    # Directly print the values of the sensor.
    print("Distance Sensor Reading: {} mm ".format(my_distance_sensor.read_mm()))
