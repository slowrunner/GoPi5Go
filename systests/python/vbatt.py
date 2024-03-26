#!/usr/bin/env python3

from easygopigo3 import EasyGoPiGo3
from time import sleep
from statistics import mean

egpg = EasyGoPiGo3(use_mutex=True)

DIODE_DROP = 0.76

x = []

for i in [1,2,3]:
    sleep(0.005)
    x += [egpg.volt()]
out = mean(x) + DIODE_DROP

print("Battery: {:.2f} volts".format(out))

