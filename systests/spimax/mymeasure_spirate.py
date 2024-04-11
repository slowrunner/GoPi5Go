#!/usr/bin/env python3

# FILE:  mymeasure_spirate.py

# Calculates fastest GoPiGo3 API get_motor_encoder() rate
#
# SPECIAL - using slower SPI max transfer rate
"""
RESULT: 

Calling get_motor_encoder() - ctrl-c to end test
^C
Called get_motor_encoder() 107827 times in 31.755 sec
get_motor_encoder(): 0.0003 sec at 3396 Hz
Done
"""

import sys
sys.path.append('/home/pi/GoPi5Go/plib')
# from noinit_easygopigo3 import EasyGoPiGo3
from myeasygopigo3 import EasyGoPiGo3
import lifeLog

from time import perf_counter
from time import sleep

def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)
    count = 0
    start_pc = perf_counter()
    try:
        print("Calling get_motor_encoder() - ctrl-c to end test")
        while True:
            left_pos = egpg.get_motor_encoder(egpg.MOTOR_LEFT)
            count += 1
    except KeyboardInterrupt:
        end_pc = perf_counter()
        dur_secs = (end_pc - start_pc)
        dur_call = dur_secs/count
        print("\nCalled get_motor_encoder() {:d} times in {:.3f} sec".format(count,dur_secs))
        print("get_motor_encoder(): {:.4f} sec at {:.0f} Hz".format(dur_call, 1/dur_call))
    except Exception as e:
        str_to_log="Exception:"+type(e).__name__+": "+str(e)
        print(str_to_log)
        lifeLog.logger.info(str_to_log)

    finally:
        print("Done")


if __name__ == '__main__':
    main()
