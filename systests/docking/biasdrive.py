#!/usr/bin/env python3

# FILE:  biasdrive.py
#
# PURPOSE:  drive with bias to correct tendancy to drift off straight
#
# API:  drive_cm_bias(dist,bias,speed,egpg) # dist cm, bias and speed m/s
#

"""
   USAGE:  execute ./biasdrive.py to test 1 meter at 0.1m/s
"""


import sys
sys.path.append('/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
from time import sleep

DEBUG = False

def drive_cm_bias(dist,bias,speed,egpg):
            # x_dps = int(SPEED * 1000  * 360.0 / egpg.WHEEL_CIRCUMFERENCE)
            # print("Setting Speed To {:.2f} m/s {:d} DPS".format(SPEED,x_dps))
            # egpg.set_speed(x_dps)

            right_speed = (speed * 1000) + bias * egpg.WHEEL_BASE_WIDTH / 2
            left_speed  = (speed * 1000) - bias * egpg.WHEEL_BASE_WIDTH / 2
            right_dps  = int( right_speed * 360.0 / egpg.WHEEL_CIRCUMFERENCE)
            left_dps   = int( left_speed  * 360.0 / egpg.WHEEL_CIRCUMFERENCE)
            drive_time = dist / (speed * 100.00)   # cm / (m/s * 100)
            if DEBUG:
                print("Drive Parameters - Speeds(L/R): ({:.3f},{:.3f}) m/s   DPS: ({:d},{:d})  Time: {:.1f}s".format(
                    left_speed/1000,right_speed/1000,left_dps,right_dps,drive_time))
                print("Resetting Encoders")
            egpg.reset_encoders()
            sleep(0.005)
            startposleft  = egpg.get_motor_encoder(egpg.MOTOR_LEFT)
            startposright = egpg.get_motor_encoder(egpg.MOTOR_RIGHT)
            if DEBUG: print("- encoder startpos: ({:d},{:d})".format(startposleft,startposright))
            wheelturndeg  = int ((dist * 10.0 / egpg.WHEEL_CIRCUMFERENCE) * 360)
            wheelturndeg_l = int(left_dps * drive_time)
            wheelturndeg_r = int(right_dps * drive_time)

            if DEBUG: print("- wheel turn deg: {:d}  w/bias: ({:d},{:d}) deg".format(wheelturndeg,wheelturndeg_l,wheelturndeg_r))
            egpg.set_motor_limits(egpg.MOTOR_LEFT+egpg.MOTOR_RIGHT,0)  # allow max speed possible
            egpg.set_motor_dps(egpg.MOTOR_LEFT, left_dps)
            egpg.set_motor_dps(egpg.MOTOR_RIGHT, right_dps)
            sleep(0.1)  # allow motors to start turning

            while egpg.target_reached(
                    startposleft + wheelturndeg_l,
                    startposright + wheelturndeg_r) is False:
                lp=egpg.get_motor_encoder(egpg.MOTOR_LEFT)
                rp=egpg.get_motor_encoder(egpg.MOTOR_RIGHT)
                # print("encoders: ({:d},{:d})".format(lp,rp))
                sleep(0.01)
            egpg.stop()
            if DEBUG:
                print("End Of Driving")
                sleep(2)
                lp=egpg.get_motor_encoder(egpg.MOTOR_LEFT)
                rp=egpg.get_motor_encoder(egpg.MOTOR_RIGHT)
                print("encoders: ({:d},{:d})".format(lp,rp))

                left_distance = egpg.get_motor_encoder(egpg.MOTOR_LEFT) * egpg.WHEEL_CIRCUMFERENCE / 360.0
                right_distance = egpg.get_motor_encoder(egpg.MOTOR_RIGHT) * egpg.WHEEL_CIRCUMFERENCE / 360.0
                ave_distance = (left_distance + right_distance)/2.0
                print("Encoder Distances - Ave: {:.1f}  Left: {:.1f}  Right: {:.1f} mm".format(ave_distance,left_distance, right_distance))


def main():

    SPEED = 0.1  # m/s
    BIAS = 0.01  # m/s  add angular to make drive straight
    dist_cm = 100

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    try:
        drive_cm_bias(dist_cm, BIAS, SPEED, egpg)
    except KeyboardInterrupt:
        egpg.stop()
        print("\nTest Terminated")
    finally:
        egpg.stop()
        sleep(5)
        print("Outta Here...")

if __name__ == '__main__':
    main()
