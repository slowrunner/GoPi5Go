#!/usr/bin/python3

# spi_mutex_user.py    Test mutex protected SPI for SPI errors with battery voltage



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
from spi_mutex_gopigo3 import GoPiGo3

DIODE_DROP = 0.76

def getUptime():
    res = os.popen('uptime').readline()
    return res.replace("\n", "")

def printStatus(gpg,loopCnt):
    print("\n********* (user1) GoPi5Go-Dave MUTEX Protected SPI TEST {:d} STATUS *****".format(loopCnt))
    print("{} {}".format(datetime.now().date(), getUptime()))
    vcc = gpg.get_voltage_battery()+DIODE_DROP
    print("Battery: %0.2f volts" % vcc)
    print("spi_errors: {:d}".format(gpg.spi_errors))

# ##### MAIN ######


def handle_ctlc():
    print("status.py: handle_ctlc() executed")


def main():
    # #### SET CNTL-C HANDLER
    myPyLib.set_cntl_c_handler(handle_ctlc)

    # #### Create a mutex protected instance of EasyGoPiGo3 base class
    # egpg = EasyGoPiGo3(use_mutex=True,noinit=True)
    gpg = GoPiGo3()
    loopFlag = True

    strToLog = "Starting spitest.py - "
    print(strToLog)
    loopCnt = 0

    try:
        while True:
            time.sleep(.001)
            loopCnt+=1
            if (loopCnt % 5000) == 1:  
                printStatus(gpg,loopCnt)
            else:
                vcc = gpg.get_voltage_battery()+DIODE_DROP

            if (loopFlag is False):
                break
        # end while
    except SystemExit:
        strToLog = "Exiting spitest.py - "
        print(strToLog)
        time.sleep(1)


if __name__ == "__main__":
    main()
