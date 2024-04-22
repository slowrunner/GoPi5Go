#!/usr/bin/env python3

import ina219
from ina219 import DeviceRangeError
import time
import logging
import statistics

SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 2.0

ina = ina219.INA219(SHUNT_OHMS,MAX_EXPECTED_AMPS, log_level=logging.ERROR)
ina.configure(ina.RANGE_16V,bus_adc=ina.ADC_128SAMP,shunt_adc=ina.ADC_128SAMP)
time.sleep(0.5)

# ina = ina219.INA219(SHUNT_OHMS)
# ina.configure(ina.RANGE_16V)

power_meter = 0
measurement_count = 0
mAh_meter = 0

rate_per_hour = 20

def ave_voltage():
        vlist = []
        for i in range(3):
            vBatt = ina.voltage()
            vlist += [vBatt]
            time.sleep(0.01)  # cannot be faster than 0.005
        return statistics.mean(vlist)

def ave_current():
        clist = []
        for i in range(3):
            cBatt = ina.current()
            clist += [cBatt]
            time.sleep(0.01)  # cannot be faster than 0.005
        return statistics.mean(clist)

def ave_power():
        plist = []
        for i in range(3):
            pBatt = ina.power()
            plist += [pBatt]
            time.sleep(0.01)  # cannot be faster than 0.005
        return statistics.mean(plist)/1000.0


try:
    current_now = ina.current()
    if (current_now < 0):
        charging = True
    else:
        charging = False
    last_charging = charging

    while True:
        current_now = ave_current()
        voltage_now = ave_voltage()
        power_now = ave_power()
        if (current_now < 0):
            charging = True
        else:
            charging = False

        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        if not (charging == last_charging):
            if (last_charging == True):
                print("{} Charge Complete: {:.0f} mAh  {:.1f}Wh \n".format(tnow,mAh_meter,power_meter))
            else:
                print("{} Playtime Complete: {:.0f} mAh  {:.1f}Wh \n".format(tnow,mAh_meter,power_meter))
            mAh_meter = 0
            power_meter = 0
            last_charging = charging
        mAh_meter += current_now / rate_per_hour 
        power_meter += power_now / rate_per_hour

        print("{} Reading: {:.2f} V  {:.3f} A  {:.2f} W  {:.0f} mAh  {:.1f}Wh     ".format(tnow,ave_voltage(), ave_current()/1000.0, ave_power(),mAh_meter,power_meter))
        time.sleep(3600/rate_per_hour)
except DeviceRangeError as e:
    print(e)
finally:
    ina.wake()
