#!/usr/bin/env python3
from ina219 import INA219
from ina219 import DeviceRangeError
import logging
import time

SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 2.0

def read():
    ina = INA219(SHUNT_OHMS,MAX_EXPECTED_AMPS, log_level=logging.ERROR)
    ina.configure(bus_adc=ina.ADC_128SAMP,shunt_adc=ina.ADC_128SAMP)
    time.sleep(0.1)

    try:
        print("Bus Voltage: %.2f v" % ina.voltage())
        print("Bus Current: %.3f mA" % ina.current())
        print("Power: %.3f mW" % ina.power())
        # print("Shunt voltage: %.3f mV" % ina.shunt_voltage())
    except DeviceRangeError as e:
        # Current out of device range with specified shunt resistor
        print(e)


if __name__ == "__main__":
    read()
