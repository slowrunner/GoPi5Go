#!/usr/bin/env python3

# FILE:  drive_gopigo3_keyboard.py

# Drive GOPIGO3 robot from keyboard with EasyGoPiGo3 API
# (This one uses my custom noinit_easygopigo3.py)

import sys
sys.path.append('/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
from time import sleep
import math

import termios
import tty


msg = """
This program takes keypresses from the keyboard to drive the GoPiGo3 robot
using the EasyGoPiGo3 API.  It works best with a US keyboard layout.
---------------------------
Moving around:

forward        i
spin ccw   j   k   l   spin cw
backward       ,

STOP:  k or spacebar

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1,  0),
    'j': (0,  1),
    'l': (0, -1),
    ',': (-1, 0),
    ' ': (0, 0),
    'k': (0, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn, egpg):
    turn_degpersec = turn/6.283185 * 360.0
    x_dps = int(speed * 1000  * 360.0 / egpg.WHEEL_CIRCUMFERENCE)
    return 'currently:\tspeed {:.2f} m/s ({} DPS) \tturn {:.2f} rad/s {:.0f} deg/s'.format(speed, x_dps, turn, turn_degpersec)


def main():
    settings = saveTerminalSettings()
    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    speed = 0.1  # m/s
    turn = 1.0   # rad/s
    x = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed, turn, egpg))
        while True:
            key = getKey(settings)
            # print("key: ",key)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed, turn, egpg))
                if (status == 5):
                    print(msg)
                    print(vels(speed, turn, egpg))
                status = (status + 1) %  6
            elif (key == '\x03'):
                    break
            else:
                egpg.stop()

            # dps = speed (m/s) * 1000 (mm/m) * 360 (deg) / wheel_circumference (mm)
            dps = int(speed * 1000  * 360.0 / egpg.WHEEL_CIRCUMFERENCE)
            spin_dps = int(turn * egpg.WHEEL_BASE_WIDTH / egpg.WHEEL_DIAMETER * 360 / (2*math.pi))
            # print("x_dps: {} DPS   spin_dps: {} DPS".format(dps, spin_dps))

            if key in moveBindings.keys():
                if (x > 0):
                    if (th == 0):
                        egpg.set_speed(dps)
                        egpg.forward()
                elif (x<0):
                    if (th == 0):
                        egpg.set_speed(dps)
                        egpg.backward()
                else:   # x == 0
                    if (th > 0):
                        egpg.set_speed(spin_dps)
                        egpg.spin_left()
                    elif (th < 0):
                        egpg.set_speed(spin_dps)
                        egpg.spin_right()
                    else:
                        egpg.stop()


    except Exception as e:
        print(e)

    finally:
        egpg.stop()
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
