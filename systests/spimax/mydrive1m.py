#!/usr/bin/env python3

# FILE:  mydrive1m.py
#
# SPECIAL - using slower SPI max transfer rate


"""
   USAGE:  execute ./mydrive1m.py
"""


import sys
sys.path.append('/home/pi/GoPi5Go/plib')
from myeasygopigo3 import EasyGoPiGo3
from time import sleep

SPEED = 0.1  # m/s
BIAS = 0.01  # m/s  add angular to make drive straight
dist_mm = 1000

def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    try:
            # x_dps = int(SPEED * 1000  * 360.0 / egpg.WHEEL_CIRCUMFERENCE)
            # print("Setting Speed To {:.2f} m/s {:d} DPS".format(SPEED,x_dps))
            # egpg.set_speed(x_dps)

            right_speed = (SPEED * 1000) + BIAS * egpg.WHEEL_BASE_WIDTH / 2
            left_speed  = (SPEED * 1000) - BIAS * egpg.WHEEL_BASE_WIDTH / 2
            right_dps  = int( right_speed * 360.0 / egpg.WHEEL_CIRCUMFERENCE)
            left_dps   = int( left_speed  * 360.0 / egpg.WHEEL_CIRCUMFERENCE)
            drive_time = dist_mm / (SPEED * 1000.00)
            print("Drive Parameters - Speeds(L/R): ({:.3f},{:.3f}) m/s   DPS: ({:d},{:d})  Time: {:.1f}s".format(
                  left_speed/1000,right_speed/1000,left_dps,right_dps,drive_time))
            print("Resetting Encoders")
            egpg.reset_encoders()
            print("Drive 1m")
            sleep(1)
            # egpg.drive_cm(100)
            startposleft  = egpg.get_motor_encoder(egpg.MOTOR_LEFT)
            startposright = egpg.get_motor_encoder(egpg.MOTOR_RIGHT)
            print("- encoder startpos: ({:d},{:d})".format(startposleft,startposright))
            wheelturndeg  = int ((dist_mm / egpg.WHEEL_CIRCUMFERENCE) * 360)
            wheelturndeg_l = int(left_dps * drive_time)
            wheelturndeg_r = int(right_dps * drive_time)

            print("- wheel turn deg: {:d}  w/bias: ({:d},{:d}) deg".format(wheelturndeg,wheelturndeg_l,wheelturndeg_r))
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
            print("End Of Driving")
            sleep(2)
            lp=egpg.get_motor_encoder(egpg.MOTOR_LEFT)
            rp=egpg.get_motor_encoder(egpg.MOTOR_RIGHT)
            print("encoders: ({:d},{:d})".format(lp,rp))

            left_distance = egpg.get_motor_encoder(egpg.MOTOR_LEFT) * egpg.WHEEL_CIRCUMFERENCE / 360.0
            right_distance = egpg.get_motor_encoder(egpg.MOTOR_RIGHT) * egpg.WHEEL_CIRCUMFERENCE / 360.0
            print("Encoder Distances - Left: {:.1f}  Right: {:.1f} mm".format(left_distance, right_distance))
    except KeyboardInterrupt:
        egpg.stop()
        print("\nTest Terminated")
    finally:
        egpg.stop()
        sleep(5)
        print("Outta Here...")

if __name__ == '__main__':
    main()
