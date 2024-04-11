#!/usr/bin/env python3

# FILE:  test_docking.py

# Drive GOPIGO3 robot from keyboard with EasyGoPiGo3 API
# reporting average vBatt, delta average vBatt, max delta average vBatt
# (This one uses my custom noinit_easygopigo3.py)

import sys
sys.path.append('/home/pi/GoPi5Go/plib')
# from noinit_easygopigo3 import EasyGoPiGo3
from myeasygopigo3 import EasyGoPiGo3
import mybattery as battery


from time import sleep
import math
import statistics

def aveBatteryV(egpg,vlist):
    for i in range(3):
        vBatt = battery.vBatt_vReading(egpg)[0]
        # if vBatt is obviously out of range, read again
        if (vBatt < 9):  
            sleep(0.01)
            vBatt = battery.vBatt_vReading(egpg)[0]
        vlist += [vBatt]
        sleep(0.01)  # cannot be faster than 0.005
    if ( len(vlist)>9 ):
       # print("vlist before shortening: ",vlist)
       vlist = vlist[-9:]
       # print("vlist after shortening: ",vlist)
    return statistics.mean(vlist[-3:]),vlist

def delta_ave_vBatt(vlist):
    # print("vlist ", vlist)
    # print("vlist[-3:] ",vlist[-3:], "vlist[-9:-6] ", vlist[-9:-6])
    res = statistics.mean(vlist[-3:]) - statistics.mean(vlist[-9:-6])
    return res

def ave_vBatt_str(vBatt,deltavBatt,maxDeltavBatt):
    return "vbatt: {:>5.2f}  d(vBatt): {:>6.3f}  max d(vBatt): {:>6.3f}".format(vBatt,deltavBatt,maxDeltavBatt)




def main():
    global egpg

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    vlist = []
    vBatt,vlist = aveBatteryV(egpg,vlist)
    vBatt,vlist = aveBatteryV(egpg,vlist)
    vBatt,vlist = aveBatteryV(egpg,vlist)
    davevBatt = delta_ave_vBatt(vlist)
    max_d_ave_vBatt = delta_ave_vBatt(vlist)

    try:
        while True:
            vBatt,vlist = aveBatteryV(egpg,vlist)
            davevBatt = delta_ave_vBatt(vlist)
            max_d_ave_vBatt = max(max_d_ave_vBatt,davevBatt)
            if (max_d_ave_vBatt > 1.0):
                print("resetting max_d_ave_vBatt: {:.3f}".format(max_d_ave_vBatt))
                max_d_ave_vBatt = 0
            print(ave_vBatt_str(vBatt,davevBatt,max_d_ave_vBatt)+"             ",end = "\r")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

    finally:
        print("\n")


if __name__ == '__main__':
    main()
