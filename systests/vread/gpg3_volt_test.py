#!/usr/bin/python3

# FILE: gpg3_volt_test.py

# PURPOSE: Characterize GoPiGo3 battery voltage measurements

# RESULTS:
#  - Taking readings more often than 0.002 seconds results in frequent bad data and SPI No Response errors
#  - bad data usually 0.045v or 0.046v

"""

  USAGE: ./gpg3_volt_test.py [-h] [-i INTERVAL]

  optional arguments:
    -h, --help            show this help message and exit
    -i INTERVAL, --interval INTERVAL (n.n in seconds, default=1.0)

  OUTPUT: file ./volt_results.txt
"""

import sys
from easygopigo3 import EasyGoPiGo3
from time import sleep
import argparse
import statistics

argparser = argparse.ArgumentParser(description='gpg3_volt_test.py Characterize GoPiGo3 battery voltage measurements')
argparser.add_argument('-i', '--interval', dest='interval', type=float, help="n.n in seconds, default=1.0",default=1.0)
argparser.add_argument('-p', '--print', dest='console_out', type=bool, help="print to console as measurements are taken",default=False)

args = argparser.parse_args()
sleep_interval_seconds = args.interval
console_out = args.console_out

mybot = EasyGoPiGo3()

# value = 0
values = []
count = 0
# Reference_Input_Voltage = 12.0
Reference_Input_Voltage = mybot.get_voltage_battery()

file1 = open("./volt_results.txt", "a")

def round_up(x, decimal_precision=2):

#  "x" is the value to be rounded using 4/5 rounding rules
#  always rounding away from zero
#
#  "decimal_precision is the number of decimal digits desired
#  after the decimal divider mark.
#
#  It returns the **LESSER** of:
#     (a) The number of digits requested
#     (b) The number of digits in the number if less
#         than the number of decimal digits requested
#     Example:  (Assume decimal_precision = 3)
#         round_up(1.123456, 3) will return 1.123. (4 < 5)
#         round_up(9.876543, 3) will return 9.877. (5 >= 5)
#         round_up(9.87, 3) will return 9.87
#         because there are only two decimal digits and we asked for 3
#
    if decimal_precision < 0:
        decimal_precision = 0

    exp = 10 ** decimal_precision
    x = exp * x

    if x > 0:
        val = (int(x + 0.5) / exp)
    elif x < 0:
        val = (int(x - 0.5) / exp)
    else:
        val = 0

    if decimal_precision <= 0:
        return (int(val))
    else:
        return (val)


try:
    print("\ngpg3_volt_test with read interval {:.4f} seconds".format(sleep_interval_seconds))
    spi_error = 0
    bad_return = 0
    while True:
        # Measured_Battery_Voltage =  round_up(mybot.get_voltage_battery(), 3)
        try:
            Measured_Battery_Voltage =  mybot.get_voltage_battery()
            if Measured_Battery_Voltage <= 1:
                bad_return += 1
                print("\nBad Return Count: {} Value: {:.3f}".format(bad_return, Measured_Battery_Voltage))
                continue
            # Five_v_System_Voltage = round_up(mybot.get_voltage_5v(), 3)
            # Measured_voltage_differential =  round_up((Reference_Input_Voltage - Measured_Battery_Voltage),3)
            Measured_voltage_differential =  Reference_Input_Voltage - Measured_Battery_Voltage
            # value = value + Measured_voltage_differential
            values.append(Measured_voltage_differential)
            count = count+1
            if console_out:
                print("\nMeasured Battery Voltage =", Measured_Battery_Voltage)
                print("Measured voltage differential = {:.3f}".format( Measured_voltage_differential))
                # print("5v system voltage =", Five_v_System_Voltage, "\n")
                print("Total number of measurements so far is ", count)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            spi_error += 1
            str_to_log="\nException "+type(e).__name__+": "+str(e)+" count: {}".format(spi_error)+" continuing"
            print(str_to_log)

        sleep(sleep_interval_seconds)

except KeyboardInterrupt:
    print("\nThat's All Folks!\n")
    data="\nWe took " + str(count) + " measurements at {:.4f}".format(sleep_interval_seconds) +  " seconds (based on reference voltage of " +  str(Reference_Input_Voltage) + ")"
    print(data)
    file1.write(data)
    file1.write("\n")
    data="min: {:.3f}  max: {:.3f}  mean: {:.3f}  mode: {:.3f}   sdev: {:.3f}    bad_data: {}  SPI_errors: {}\n".format(min(values), max(values), statistics.mean(values), statistics.mode(values), statistics.pstdev(values), bad_return, spi_error)
    print(data)
    file1.write(data)
    file1.close()
    sys.exit(0)
