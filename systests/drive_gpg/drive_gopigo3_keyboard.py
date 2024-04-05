#!/usr/bin/env python3

# FILE:  drive_gopigo3_keyboard.py

# Drive GOPIGO3 robot from keyboard with EasyGoPiGo3 API

import sys
sys.path.append('/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
from time import sleep

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard to drive the GoPiGo3 robot
using the EasyGoPiGo3 API.  It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

space bar: stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    ' ': (0, 0, 0, 0),
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
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()
    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)
    egpg.set_speed(125)
    # print("NO_LIMIT_SPEED:",egpg.NO_LIMIT_SPEED)

    speed = 0.1  # m/s
    turn = 1.0   # rad/s
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if (key == ' '):
                print("key: spacebar")
            else:
                print("key: ",key)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                print("x: {:.2f} y: {:.2f} z: {:.2f} th: {:.2f}".format(x,y,z,th))
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif (key == '\x03'):
                    break
            else:
                pass

            if key in moveBindings.keys():
                if (x > 0):
                    egpg.forward()
                elif (x<0):
                    egpg.backward()
                else:
                    egpg.stop()

            # twist = geometry_msgs.msg.Twist()
            # twist.linear.x = x * speed
            # twist.linear.y = y * speed
            # twist.linear.z = z * speed
            # twist.angular.x = 0.0
            # twist.angular.y = 0.0
            # twist.angular.z = th * turn
            # pub.publish(twist)


    except Exception as e:
        print(e)

    finally:

        # twist = geometry_msgs.msg.Twist()
        # twist.linear.x = 0.0
        # twist.linear.y = 0.0
        # twist.linear.z = 0.0
        # twist.angular.x = 0.0
        # twist.angular.y = 0.0
        # twist.angular.z = 0.0
        # pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
