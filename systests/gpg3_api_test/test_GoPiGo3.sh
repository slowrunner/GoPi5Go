#!/bin/bash

python /home/pi/Dexter/GoPiGo3/Software/Python/Examples/Read_Info.py
echo "PRESS CTRL-C AFTER LEDS BLINK"
python3 /home/pi/Dexter/GoPiGo3/Software/Python/Examples/LED.py
echo "PRESS CTRL-C AFTER SERVO MOVES"
python3 /home/pi/Dexter/GoPiGo3/Software/Python/Examples/Servo.py
echo "PRESS CTRL-C AFTER ROBOT RETURNS TO FACE FORWARD"
python3 /home/pi/Dexter/GoPiGo3/Software/Python/Examples/Motor_Turn.py
echo "PRESS CTRL-C AFTER ULTRASONIC RANGER TEST"
python ~/Dexter/GoPiGo3/Software/Python/Examples/Grove_US2.py
echo "PRESS CTRL-C TO END DISTANCE SENSOR TEST"
python3 /home/pi/Dexter/GoPiGo3/Software/Python/Examples/easy_Distance_Sensor.py

