# INA219 I2C Voltage and Current Sensor on Raspberry Pi 5

* Install:

```
git clone https://github.com/slowrunner/gopigo3-pi5-ina219.git
cd gopigo3-pi5-ina219/
sudo python3 setup.py install
./example.py 

pi@GoPi5Go:~/GoPi5Go/systests/ina219 $ ./example.py 
2024-04-17 00:14:51,732 - INFO - INA219 gain set to 0.04V
2024-04-17 00:14:51,732 - INFO - INA219 calibrate called with: bus max volts: 16V, max shunt volts: 0.04V, max expected amps: 0.200A
2024-04-17 00:14:51,732 - INFO - INA219 max possible current: 0.400A
2024-04-17 00:14:51,732 - INFO - INA219 max expected current: 0.200A
2024-04-17 00:14:51,732 - INFO - INA219 current LSB: 6.250e-06 A/bit
2024-04-17 00:14:51,732 - INFO - INA219 power LSB: 1.250e-04 W/bit
2024-04-17 00:14:51,732 - INFO - INA219 max current before overflow: 0.2048A
2024-04-17 00:14:51,732 - INFO - INA219 max shunt voltage before overflow: 20.4800mV
2024-04-17 00:14:51,732 - INFO - INA219 calibration: 0xfffe (65534)
Bus Voltage    : 12.308 V
Bus Current    : 10.100 mA
Supply Voltage : 12.285 V
Shunt voltage  : 0.550 mV
Power          : 12.250 mW


```

* Confirm visible on I2C
```
i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         08 -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
``` 

For complete info, see [gopigo3-pi5-ina219/README.md](https://github.com/slowrunner/gopigo3-pi5-ina219/blob/main/README.md)

Specifications (Adafruit INA219 High Side DC Current Sensor Breakout - 2018 era version):
* 0.1 ohm 1% 2W current sense resistor
* Up to +26V target voltage
* Up to ±3.2A current measurement, with ±0.8mA resolution
