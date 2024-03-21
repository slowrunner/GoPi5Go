# The GoPiGo3 C++ API

```/home/pi/Dexter/GoPiGo3/Software/C/``` contains the current official GoPiGo3 C++ API  
in files GoPiGo3.h and GoPiGo3.cpp (which end up as the .h and libgopigo3.so for linking)  

This folder mimicks the /home/pi/Dexter/GoPiGo3/Software/C/ folder  
of the official GoPiGo3 Github repository, with the following differences:  
- executables are placed in ..C/build/  (CMakeLists.txt change)  
- new folder robot/ contains a test robot derived from Examples/drive.cpp  


### Setup:  
- sudo apt install cmake  
- cmake CMakeLists.txt  
- make  
- [make install] will put:  
  - /usr/local/include/GoPiGo3.h  
  - /usr/local/lib/libgopigo3.so  

### Other Useful Commands:  
- ```make clean``` will remove all executables from build/  
- ```make``` will rebuild any changed project  

### To run Examples (after make):  
- ```build/info```        - reads GoPiGo3 red board information and reports battery voltage  
- ```build/vbatt```       - reports battery voltage reading and VCC (5v) voltage   
                            Actual battery voltage is 0.8v higher than reported  
                            due to reverse polarity protection diode drop.  
- ```build/leds```        - cycles center "WiFi LED" through colors  
- ```build/servos```      - centers properly attached servos  
                            (Servo1: brown wire closest to front of robot)  
                            (Servo2: brown wire closest to back of robot)  
- ```build/motors```      - Gently rotate robot's left wheel in a forward direction  
                            Robot's right wheel will rotate to match left wheel encoder value
- ```build/sensors```     - Reports Grove Ultrasonic Ranger (connected to AD1) range in millimeters  
                            Reports Infrared Remote Control (connected to AD2) code  
- ```build/i2c```         - Toggles P0 output of PCA9570 I2C output expander connected to AD1 port ???  
- ```build/grove_led```   - Varies Grove LED brightness (connected to AD1)  
- ```build/drive```       - Allows driving GoPiGo3 with key presses  
                            spacebar: stop  
                            w: forward at 150 DPS  
                            s: spin  
                            x: backward at 150 DPS  
                            a: pivot ccw around left wheel  
                            d: pivot cw around right wheel  
                            q: exit (only q, cntrl-c does not exit)  
- ```build/robot```       - runs test robot (drive.cpp with status output if press key during motion)  
