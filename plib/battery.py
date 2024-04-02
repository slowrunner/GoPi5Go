#!/usr/bin/python3

# FILE: battery.py

# PURPOSE:  Central facts and methods for the ModRobotics/TalentCell YB1203000 Li-Ion Battery Pack
#           12.6v to 9v 3A "3000mAh" Battery Pack
#

import sys
sys.path.insert(1,"/home/pi/GoPi5Go/plib/")
# from noinit_easygopigo3 import EasyGoPiGo3
from easygopigo3 import EasyGoPiGo3
import lifeLog
import numpy as np

CHARGING_ADJUST_V = 0.2    # When charging the GoPiGo3 reading is closer to the actual battery voltage
REV_PROTECT_DIODE = 0.76    # The GoPiGo3 has a reverse polarity protection diode drop of 0.6v to 0.8v (n=2)
                            # This value results in average readings vs battery voltage error of +/- 0.03
SAFETY_SHUTDOWN_vBatt = 9.75   # Battery Discharge Protection Circuit allows down to 8.15v (9.75 leaves ~15m reserve)
SAFETY_SHUTDOWN_vReading = SAFETY_SHUTDOWN_vBatt - REV_PROTECT_DIODE   # 8.5v EasyGoPiGo3.volt() reading
WARNING_LOW_vBatt = 10.25       # Give (~15 minutes) Advance Warning before safety shutdown

VOLTAGE_POINTS = \
    [8.21, 9.489, 9.643, 9.754, 9.934, 9.969, 10.011, 10.08, 10.157, 10.243, 10.303, 10.39, 10.474, 10.594, 10.731, 10.885, 10.945, 11.108, 11.348, 11.44, 11.605, 12.4]
BATTERY_REMAINING = \
    [0.0,   0.05,  0.10,  0.15,  0.20,  0.25,   0.30,  0.35,   0.40,   0.45,   0.50,  0.55,   0.60,   0.65,   0.70,   0.75,   0.80,   0.85,   0.90,  0.95,   0.99, 1.00]

def pctRemaining_from_vBatt(vBatt):
      FULL_CHARGE = 1.0
      PROTECTION_CUTOFF = 8.21
      pctRemaining = np.interp(vBatt,VOLTAGE_POINTS,BATTERY_REMAINING, left=FULL_CHARGE, right=PROTECTION_CUTOFF)
      return pctRemaining

def vBatt_vReading(egpg):
      vReading = 0
      try:
          vReading = egpg.volt()
      except Exception as e:
          str_to_log="Exception "+type(e).__name__+": "+str(e)+" vReading:{:.2f}".format(vReading)+" continuing"
          print(str_to_log)
          lifeLog.logger.info(str_to_log)
      if (vReading > 11.7):
          # charging
          vBatt = vReading + REV_PROTECT_DIODE - CHARGING_ADJUST_V
          # print("charging")
      else:
          vBatt = vReading + REV_PROTECT_DIODE
      return vBatt,vReading

def voltages_string(egpg):
        vBatt, vReading = vBatt_vReading(egpg)
        pctRemaining = pctRemaining_from_vBatt(vBatt)*100.0
        return "Current Battery {:.1f}v  {:.0f}%  (Reading {:.2f}v)".format(vBatt,pctRemaining,vReading)

def too_low(egpg):
	vBatt, _ = vBatt_vReading(egpg)
	return vBatt < SAFETY_SHUTDOWN_vBatt

def on_last_leg(egpg):
	vBatt, _ = vBatt_vReading(egpg)
	return vBatt < WARNING_LOW_vBatt



def testMain():
	# egpg = EasyGoPiGo3(use_mutex=True, noinit=True)
	egpg = EasyGoPiGo3(use_mutex=True)  
	print(voltages_string(egpg))  
	vBatt = egpg.volt() + REV_PROTECT_DIODE
	print("Battery Percent Remaining: {:.0f}".format(pctRemaining_from_vBatt( vBatt )*100  ))


if __name__ == '__main__': testMain()
