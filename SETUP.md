# Setup GoPi5Go_Dave 

PiOS Bookworm Desktop


As Of: 19 MAR 2024 with 32-bit PiOS  

** Get Latest Raspberry Pi Imager (will list latest OS options) **  
https://www.raspberrypi.com/software/  

**Write 64-bit PiOS Bookworm Desktop to SDcard with Raspberry Pi Imager**  
  - Choose OS -> Pi5 ->  64-bit Pi OS (Bookworm) Desktop  
  - Preconfigure WiFi SSID/PW, user, pw, locale, in Imager (SSID is case sensitive)  

**Disable IPv6 to make life easier( apt updates succeed without specifying IPv4 only )  
nano cmdline.txt  add ipv6.disable=1 at end of line, do not add return at end  


**SSH:**   Setting in imager WORKED!  
 - ssh pi@X.X.X.X   
   - (If needed:  ssh-keygen -R X.X.X.X  , then try ssh again)  
   - pw: your password  


**Update OS**  

```
$ sudo apt update
$ sudo apt full-upgrade

$ sudo reboot
```


- Raspberry Pi Configuration  

```
sudo raspi-config
  - turn off auto login
  - display->headless configuration 1920x1080
  - Interfaces -> VNC, SPI, I2C
  - Localization->Set Locale: US
reboot now? yes
```

** === Bring Down GoPi5Go software  

```
cd ~
git clone https://github.com/slowrunner/GoPi5Go.git
```

** === Setup ip_feedback (and espeak-ng)  

```
cd GoPi5Go/config
./install_ip_feedback_service.sh
```


** === Setup life logging

copy crontab-e last three lines  

```
sudo crontab -e
- select nano 1
- paste three lines at end of file
- reboot
```


** === Setup Pi5 Napping  

Follow RTC.README.md  


**===Install GoPiGo3 software**  

```
./get_dexter_software.sh
```

** === Install GoPi5Go Power Management Service  

```
./install_gp5g_power_service.sh
```

** === TEST C++ GoPiGo3 API

```
cd ~/GoP5Go/systests/cpp
cmake CMakeList.txt
make
./robot

pi@GoPi5Go:~/GoPi5Go/systests/cpp $ ./robot

**** GoPiGo3 Robot Constants:
 - WHEEL_DIAMETER: 66.500 mm
 - WHEEL_BASE_WIDTH: 117.000 mm
 - ENCODER_TICKS_PER_ROTATION: 6
 - MOTOR_GEAR_RATIO: 120

****
        fwd w                  r reset encoders 
 left  a  spin s   d  right
            bkwd x 
                               spacebar  STOP
****

encoders: (0, 0) Cmd: (q to exit): w
LEFT : motor_state: 0,  pwr: 100,  pos: 1,  dps: 0
RIGHT: motor_state: 0,  pwr: 100,  pos: 0,  dps: 0

encoders: (1, 0) Cmd: (q to exit):  stopping..
LEFT : motor_state: 0,  pwr: 0,  pos: 2100,  dps: 0
RIGHT: motor_state: 0,  pwr: 0,  pos: 2043,  dps: 0

encoders: (2100, 2043) Cmd: (q to exit): s
LEFT : motor_state: 0,  pwr: 0,  pos: 2100,  dps: 0
RIGHT: motor_state: 0,  pwr: 0,  pos: 2043,  dps: 0

encoders: (2100, 2043) Cmd: (q to exit): q
LEFT : motor_state: 0,  pwr: -128,  pos: 3103,  dps: 153
RIGHT: motor_state: 0,  pwr: -128,  pos: 1038,  dps: -163
```
