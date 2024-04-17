#!/usr/bin/env python3

import ina219
from ina219 import DeviceRangeError
import time

SHUNT_OHMS = 0.1

ina = ina219.INA219(SHUNT_OHMS)
ina.configure(ina.RANGE_16V)
try:
    while True:
        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        print("{} Reading: {:.2f} V  {:.3f} A  {:.2f} W".format(tnow,ina.voltage(), ina.current()/1000.0, ina.power()/1000.0))
        ina.sleep()
        time.sleep(360)
        ina.wake()
except DeviceRangeError as e:
    print(e)
finally:
    ina.wake()
