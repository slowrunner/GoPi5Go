#!/usr/bin/python3
#
# safetyShutdown.py     SAFELY SHUTDOWN AT BATTERY 
#
#      Loop reading the battery voltage
#        UNTIL voltage stays below LOW_BATTERY_V 4 times,
#        then will force a shutdown.
#
#      Will start wifi led blinking orange ~15 minutes before safety shutdown is executed
#      when battery.SAFETY_SHUTDOWN_vBatt is 9.75v and battery.WARNING_LOW_vBatt is 10.25v
#
#      Note: Program prints actual battery voltage which is 0.81v higher than GoPiGo3 reading
#            due to reverse polarity protection diode, wires, and connections
#

import sys
import time
import signal
import os
from datetime import datetime

sys.path.insert(1,"/home/pi/GoPi5Go/plib")
# from noinit_easygopigo3 import EasyGoPiGo3
from myeasygopigo3 import EasyGoPiGo3
import mybattery as battery
import myleds as leds
import mystatus as status
import speak
import lifeLog

# ######### CNTL-C #####
# Callback and setup to catch control-C and quit program

_funcToRun=None

def signal_handler(signal, frame):
  print('\n** Control-C Detected')
  if (_funcToRun != None):
     _funcToRun()
  sys.exit(0)     # raise SystemExit exception

# Setup the callback to catch control-C
def set_cntl_c_handler(toRun=None):
  global _funcToRun
  _funcToRun = toRun
  signal.signal(signal.SIGINT, signal_handler)




# ##### MAIN ######

def handle_ctlc():
  global egpg
  egpg.reset_all()
  print("safetyShutdown.py: handle_ctlc() executed")

def main():
  global egpg

  # #### SET CNTL-C HANDLER 
  set_cntl_c_handler(handle_ctlc)

  # #### Create instance of GoPiGo3 base class 
  egpg = EasyGoPiGo3(use_mutex=True,noinit=True)

  batteryLowCount = 0
  last_leg_count = 0
  warning_led_on = False
  leds.wifi_blinker_off(egpg)

  str_to_log = "'Starting safetyShutdown.py at {:.2f} volts.'".format(egpg.volt()+battery.REV_PROTECT_DIODE)
  os.system("/home/pi/GoPi5Go/utils/logMaintenance.py " + str_to_log)

  try:
    while True:
        status.printStatus(egpg)
        if (battery.too_low(egpg)):
            batteryLowCount += 1
        else: 
            batteryLowCount = 0
            if (warning_led_on == True) and (battery.on_last_leg(egpg) == False):
                last_leg_count -= 1
                if last_leg_count < 1:
                    last_leg_count = 0
                    warning_led_on = False
                    leds.wifi_blinker_off(egpg)
                    os.system("/home/pi/GoPi5Go/utils/logMaintenance.py 'safetyShutdown: voltage warning blinker deactivated '")
        if (warning_led_on == False) and battery.on_last_leg(egpg):
            last_leg_count += 1
            if last_leg_count > 4:
                warning_led_on = True
                leds.wifi_blinker_on(egpg,color=leds.ORANGE)
                os.system("/home/pi/GoPi5Go/utils/logMaintenance.py 'safetyShutdown: voltage warning blinker activated '")
                last_leg_count = 150  # allow plenty of bouncing around the low mark
                speak.shout("15 minute warning.  Battery Voltage is {:.1f} volts.  Need to be Docked".format(egpg.volt()+battery.REV_PROTECT_DIODE))
        if (batteryLowCount > 3):
          vBatt,_ = battery.vBatt_vReading(egpg)
          print ("WARNING, WARNING, SHUTTING DOWN NOW")
          print ("BATTERY %.2f volts BATTERY LOW - SHUTTING DOWN NOW" % vBatt)
          egpg.reset_all()
          time.sleep(1)
          os.system("/home/pi/GoPi5Go/utils/logMaintenance.py 'SAFETY SHUTDOWN - BATTERY LOW'")
          str_log_voltages = "** "+battery.voltages_string(egpg)+" **"
          lifeLog.logger.info(str_log_voltages)

          time.sleep(1)
          # os.system("sudo shutdown +10")   # for testing
          os.system("sudo shutdown -h +2")
          sys.exit(0)
        time.sleep(10)    # check battery status every 10 seconds
                          # important to make four checks low V quickly
    #end while
  except SystemExit:
    print("safetyShutdown.py: exiting")

  except Exception as e:
      print("safetyShutdown.py exiting with exception: ",type(e).__name__,",",str(e))
      str_to_log="Exiting with Exception "+type(e).__name__+": "+str(e)+"\n"
      print(str_to_log)
      lifeLog.logger.info(str_to_log)
if __name__ == "__main__":
    main()



