#!/usr/bin/env python

import easy_ina219


def read():

    ina = easy_ina219.EasyINA219()

    print("Bus Current        : %.0f mA" % ina.milliamps())
    print("Ave Bus Current    : %.0f mA" % ina.ave_milliamps())
    print("Supply Voltage     : %.2f V" % ina.volts())
    print("Ave Supply Voltage : %.2f V" % ina.ave_volts())
    print("Power              : %.1f W" % ina.watts())
    print("Ave Power          : %.1f W" % ina.ave_watts())

if __name__ == "__main__":
    read()
