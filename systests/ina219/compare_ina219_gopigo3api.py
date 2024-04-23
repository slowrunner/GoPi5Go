#!/usr/bin/env python

# FILE:  compare_ina219_gopigo3api.py
"""
RESULTS: 
AstroAI DT132A meter  : 10.91 V
EGPG Voltage          : 10.911 V
INA219 Bus Voltage    : 10.800 V  0.111 v lower than egpg
INA219 Supply Voltage : 10.857 V  0.054 v lower than egpg
"""

import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
import battery
import noinit_easygopigo3

import logging
from ina219 import INA219
import time


SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 2.5


def print_values(ina,egpg):

    ina_supply_voltage = ina.supply_voltage()
    ina_bus_voltage = ina.voltage()
    egpg_voltage = egpg.volt()+battery.REV_PROTECT_DIODE
    diff_bus = egpg_voltage - ina_bus_voltage
    diff_supply = egpg_voltage - ina_supply_voltage
    print("EGPG Voltage          : %.3f V" % egpg_voltage )
    print("INA219 Bus Voltage    : %.3f V   %.3f v lower than egpg" % (ina_bus_voltage,diff_bus))
    print("INA219 Supply Voltage : %.3f V   %.3f v lower than egpg" % (ina_supply_voltage,diff_supply))

if __name__ == "__main__":
    ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, log_level=None)
    ina.configure(ina.RANGE_16V, ina.GAIN_AUTO, bus_adc=ina.ADC_128SAMP,shunt_adc=ina.ADC_128SAMP )
    egpg = noinit_easygopigo3.EasyGoPiGo3(noinit=True)

    try:
        while True:
            print("\n")
            print_values(ina,egpg)
            time.sleep(10)
    except KeyboardInterrupt:
        print("\n")
