#!/usr/bin/env python3

"""
FILE:  easy_ina219.py

DOC:   The EasyINA219 object provides mutex protected (Hardware I2C only) access to the INA219 Voltage and Current Sensor
       for GoPiGo3 robots.   (Hardware I2C only to be compatible with Raspberry Pi5 GoPiGo3 robots)

       Mutex protection allows multiple simultaneous threads/processes to use the INA219A sensor without I2C bus issues.

       Note: EasyINA219.ina219.xx() provides unprotected access to full INA219 class methods, but use is not recommended.

METHODS:
  EasyINA219 unique methods with I2C mutex protection:
      ave_volts()    : volts - average of 3 readings supply_voltage() 0.02s apart
      ave_milliamps(): mA - average of 3 readings 0.02s apart
      ave_watts()    : W  - average of 3 readings 0.02s apart

  Protected INA219 methods:
      volts()       : volts - single INA219.supply_voltage()
      milliamps():  : mA    - single INA219.current(), same as INA219.bus_current
      watts()       : Watts - single INA219.power()/1000

USAGE:
    import easy_ina219
    ina = EasyINA219()

    print("Current (bus)      : %.0f mA" % ina.milliamps())
    print("Ave Current (bus)  : %.0f mA" % ina.ave_milliamps())
    print("Supply Voltage     : %.2f V" % ina.volts())
    print("Ave Supply Voltage : %.2f V" % ina.ave_volts())
    print("Power              : %.1f W" % ina.watts())
    print("Ave Power          : %.1f W" % ina.ave_watts())

"""


from di_sensors.easy_mutex import ifMutexAcquire, ifMutexRelease
import time
import ina219
import statistics

# class EasyINA219(ina219.INA219):
class EasyINA219():
    """
    I2C Mutex protected class for the INA219 Voltage and Current Sensor
    """

    def __init__(self, use_mutex=True):
        self.descriptor = "Voltage and Current Sensor"
        self.ina219 = None  # non-mutex-protected ina219 sensor object
        self.use_mutex = use_mutex
        self.SHUNT_OHMS = 0.1
        self.MAX_EXPECTED_AMPS = 2.0


        ifMutexAcquire(self.use_mutex)
        try:
            # ina219.INA219.__init__(self,self.SHUNT_OHMS,self.MAX_EXPECTED_AMPS,log_level=None)
            self.ina219 = ina219.INA219(self.SHUNT_OHMS,self.MAX_EXPECTED_AMPS,log_level=None)
            # self.configure(self.RANGE_16V,bus_adc=self.ADC_128SAMP,shunt_adc=self.ADC_128SAMP)
            self.ina219.configure(self.ina219.RANGE_16V,bus_adc=self.ina219.ADC_128SAMP,shunt_adc=self.ina219.ADC_128SAMP)
            time.sleep(0.5)

        except Exception as e:
            raise
        finally:
            ifMutexRelease(self.use_mutex)


    def ave_volts(self):
            vlist = []
            for i in range(3):
                ifMutexAcquire(self.use_mutex)
                try:
                    # vBatt = self.supply_voltage()
                    vBatt = self.ina219.supply_voltage()
                except Exception as e:
                    print(e)
                    vBatt = 0
                finally:
                    ifMutexRelease(self.use_mutex)
                vlist += [vBatt]
                time.sleep(0.02)  # cannot be faster than 0.02
            return statistics.mean(vlist)

    def ave_milliamps(self):
            clist = []
            for i in range(3):
                # cBatt = self.current()
                cBatt = self.ina219.current()
                clist += [cBatt]
                time.sleep(0.02)  # cannot be faster than 0.02
            return statistics.mean(clist)

    def ave_watts(self):  # in Watts (not mW like ina.power())
            plist = []
            for i in range(3):
                # pBatt = self.power()
                pBatt = self.ina219.power()
                plist += [pBatt]
                time.sleep(0.02)  # cannot be faster than 0.02
            return statistics.mean(plist)/1000.0

    def volts(self):
            ifMutexAcquire(self.use_mutex)
            try:
                # vBatt = self.supply_voltage()
                vBatt = self.ina219.supply_voltage()
            except Exception as e:
                print(e)
                vBatt = 0
            finally:
                ifMutexRelease(self.use_mutex)
            return vBatt

    def milliamps(self):
            ifMutexAcquire(self.use_mutex)
            try:
                # cBatt = self.current()
                cBatt = self.ina219.current()
            except Exception as e:
                print(e)
                cBatt = 0
            finally:
                ifMutexRelease(self.use_mutex)
            return cBatt

    def watts(self):
            ifMutexAcquire(self.use_mutex)
            try:
                # pBatt = self.power()
                pBatt = self.ina219.power()
            except Exception as e:
                print(e)
                pBatt = 0
            finally:
                ifMutexRelease(self.use_mutex)
            return pBatt/1000.0


if __name__ == "__main__":
    ina = EasyINA219()

    print("Bus Current        : %.0f mA" % ina.milliamps())
    print("Ave Bus Current    : %.0f mA" % ina.ave_milliamps())
    print("Supply Voltage     : %.2f V" % ina.volts())
    print("Ave Supply Voltage : %.2f V" % ina.ave_volts())
    print("Power              : %.1f W" % ina.watts())
    print("Ave Power          : %.1f W" % ina.ave_watts())

