#!/usr/bin/python3

# status.py    Basic Status with battery voltage


#      import status provides printStatus(egpg,ds)

#      ./status.py    will print status once and exit
#
#      ./status.py -l (or -loop) will print status every 5 seconds
#
#      ./status.py -h (or --help) will print usage

#  Protected from No SPI Response Exceptions - continues

"""
********* GoPi5Go Dave STATUS *****
2021-08-05  08:33:37 up 2 min,  1 user,  load average: 1.14, 0.92, 0.38
Current Battery 12.05v EasyGoPiGo3 Reading 11.24v
5v Supply: 4.98
Processor Temp: 38.6'C
Clock Frequency: 1.30 GHz
throttled=0x0
"""


# IMPORTS
import sys
sys.path
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
import time
import signal
import os
import myPyLib
# import speak
# import myconfig
from datetime import datetime
from noinit_easygopigo3 import EasyGoPiGo3
import battery
# import myDistSensor
import lifeLog
# import runLog
import argparse
# from my_safe_inertial_measurement_unit import SafeIMUSensor
# import rosbotDataJson as rosbotData

# Return CPU temperature as a character string
def getCPUtemperature():
    res = os.popen('vcgencmd measure_temp').readline()
    return(res.replace("temp=", "").replace("\n", ""))


# Return Clock Freq as a character string
def getClockFreq():
    res = os.popen('vcgencmd measure_clock arm').readline()
    res = int(res.split("=")[1])
    if (res < 1000000000):
        res = str(res/1000000)+" MHz"
    else:
        res = '{:.2f}'.format(res/1000000000.0)+" GHz"
    return res


# Return throttled flags as a character string
#   0x10001  under-voltage 4.63v occurred / occurring
#   0x20002  freq-cap occurred / occurring
#   0x40004  Temp Throttled occurred / occurring
#   0x80008  SOFT_TEMPERATURE_LIMIT (default 60degC, boot/config.txt temp_soft_limit=70 to increase)

def getThrottled():
    res = os.popen('vcgencmd get_throttled').readline()
    return res.replace("\n", "")


def getUptime():
    res = os.popen('uptime').readline()
    return res.replace("\n", "")

def printStatus(egpg):
    print("\n********* GoPi5Go-Dave STATUS *****")
    print("{} {}".format(datetime.now().date(), getUptime()))
    print(battery.voltages_string(egpg))
    if battery.on_last_leg(egpg):
        print("WARNING - Battery Is Nearing Shutdown Voltage")
    try:
        v5V = egpg.get_voltage_5v()
    except Exception as e:
        str_to_log="Exception in get_voltage_5v()"+type(e).__name__+": "+str(e)+" continuing"
        print(str_to_log)
        lifeLog.logger.info(str_to_log)
        v5V = 0

    print("5v Supply: %0.2f" % v5V)
    print("Processor Temp: %s" % getCPUtemperature())
    print("Clock Frequency: %s" % getClockFreq())
    print("%s" % getThrottled())


# ##### MAIN ######


def handle_ctlc():
    print("status.py: handle_ctlc() executed")


def main():
    # #### SET CNTL-C HANDLER
    myPyLib.set_cntl_c_handler(handle_ctlc)

    # #### Create a mutex protected instance of EasyGoPiGo3 base class
    egpg = EasyGoPiGo3(use_mutex=True,noinit=True)
    # egpg = EasyGoPiGo3(use_mutex=True)
    # myconfig.setParameters(egpg)

    # ARGUMENT PARSER
    ap = argparse.ArgumentParser()
    ap.add_argument("-l", "--loop", default=False, action='store_true',
                    help="optional loop mode")
    args = vars(ap.parse_args())
    loopFlag = args['loop']

    strToLog = "Starting status.py - "+battery.voltages_string(egpg)
    print(strToLog)

    try:
        while True:
            time.sleep(5)
            printStatus(egpg)
            if (loopFlag is False):
                break
        # end while
    except SystemExit:
        strToLog = "Exiting status.py - "+battery.voltages_string(egpg)
        print(strToLog)
        time.sleep(1)


if __name__ == "__main__":
    main()
