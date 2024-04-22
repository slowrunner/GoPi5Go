#!/usr/bin/env python3

import ina219
from ina219 import DeviceRangeError
import time
import logging

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

try:
    current_now = ina.current()
    if (current_now < 0):
        charging = True
    else:
        charging = False
    last_charging = charging

    while True:
        current_now = ina.current()
        voltage_now = ina.voltage()
        power_now = ina.power()
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
        power_meter += power_now / rate_per_hour / 1000.0

        print("{} Reading: {:.2f} V  {:.3f} A  {:.2f} W  {:.0f} mAh  {:.1f}Wh     ".format(tnow,ina.voltage(), ina.current()/1000.0, ina.power()/1000.0,mAh_meter,power_meter))
        time.sleep(3600/rate_per_hour)
except DeviceRangeError as e:
    print(e)
finally:
    ina.wake()
