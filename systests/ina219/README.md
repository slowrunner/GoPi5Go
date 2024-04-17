# INA219 I2C Voltage and Current Sensor on Raspberry Pi 5

* Install:

```
git clone https://github.com/slowrunner/gopigo3-pi5-ina219.git
cd gopigo3-pi5-ina219/
sudo python3 setup.py install
./example.py 
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
