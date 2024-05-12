# Setup 2.4GHz Wireless USB Logitech F710 Game Controller

1) Plug in USB WiFi Game Controller adapter

```
Bus 001 Device 003: ID 046d:c21f Logitech, Inc. F710 Wireless Gamepad [XInput Mode]
```

## Don't need this to be installed, but it allows testing before solving the "joystick in docker" configuration

```

- pi user must belong to group input
- /dev/input/event* must belong to group input  (sudo chgrp input /dev/input/event*)

sudo apt install joystick

jstest --normal /dev/input/js0

pi@WaLiPi5:~ $ jstest --normal /dev/input/js0
Driver version is 2.1.0.
Joystick (Logitech Gamepad F710) has 8 axes (X, Y, Z, Rx, Ry, Rz, Hat0X, Hat0Y)
and 11 buttons (BtnA, BtnB, BtnX, BtnY, BtnTL, BtnTR, BtnSelect, BtnStart, BtnMode, BtnThumbL, BtnThumbR).
Testing ... (interrupt to exit)
Axes:  0:     0  1:     0  2:-32767  3:     0  4:     0  5:-32767  6:     0  7:     0 
Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 

Button 0: "A"
Button 1: "B"
Button 2: "X"
Button 3: "Y"
Button 4: Left Finger "LB"
Button 5: Right Finger "RB"
Button 6: Back
Button 7: Start
Button 8: "Logitech"
Button 9:
Button 10:

Axes 0: Left Joystick Left  (-32767) Right (+32767)
Axes 1: Left Joyttick Up    (-32767) Down (+32767)
Axes 2: Left Trigger "LT" Not-Depressed (-32767) Fully-Depressed (+32767)
Axes 3: Right Joystick Left (-32767) Right (+32767)
Axes 4: Right Joystick Up   (-32767) Down (+32767)
Axes 5: Right Trigger "RT" Not-Depressed (-32767) Fully-Depressed (+32767)
Axes 6: Rocker Left (-32767) Right (+327670
Axis 7: Rocker Up   (-32767) Down  (+32767)

```
Created F710.config.yaml:  

```
teleop_twist_joy_node:
  ros__parameters:
    axis_linear:  # Left thumb stick vertical
      x: 1
    scale_linear:
      x: 0.150
    scale_linear_turbo:
      x: 0.300   # Turbo = slow in this config.  Any more than 0.19 causes arc to right

    axis_angular:  # Left thumb stick horizontal
      yaw: 0
    scale_angular:
      yaw: 0.25
    scale_angular_turbo:
      yaw: 0.5     # slower turning for when going forward fast

    enable_button: 4  # Left Front button
    enable_turbo_button: 5  # Right Front button
```
- Add to docker_files/ros2_humble_desktop_plus_dockerfile  

```
# files to copy must be in the docker tree
COPY snes_slow.config.yaml /opt/ros/humble/share/teleop_twist_joy/config/
COPY F710.config.yaml /opt/ros/humble/share/teleop_twist_joy/config/
RUN sudo chgrp input /dev/input/event*

```

Try:
```
$ ros2 run joy joy_enumerate_devices 
ID : GUID                             : GamePad : Mapped : Joystick Device Name
-------------------------------------------------------------------------------
 0 : 030000006d0400001fc2000005030000 :    true :   true : Logitech F710 Gamepad (XInput)
```

# Setup 2.4GHz Wireless USB SNES Gamepad


1) Plugged in USB WiFi Gamepad adapter

```
$ lsusb
  ...
  Bus 001 Device 005: ID 0079:0126 DragonRise Inc.
```

# Don't need this to be installed, but it allows testing before solving the "joystick in docker" configuration

```
sudo apt install joystick

jstest --normal /dev/input/js0

Driver version is 2.1.0.
Joystick (Controller) has 6 axes (X, Y, Z, Rz, Hat0X, Hat0Y)
and 13 buttons (BtnA, BtnB, BtnC, BtnX, BtnY, BtnZ, BtnTL, BtnTR, BtnTL2, BtnTR2, BtnSelect, BtnStart, BtnMode).
Testing ... (interrupt to exit)
Axes:  0:     0  1:     0  2:     0  3:     0  4:     0  5:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 11:off 12:off 


Pad Up:    Axis 1: -32767
Pad Dn:    Axis 1:  32767

Pad Left:  Axis 0: -32767
Pad Right: Axis 0:  32767

Select: Button 8
Start:  Button 9

X: Button 0
B: Button 2
Y: Button 3
A: Button 1

Forefinger pads:
Left:  Button 4
Right: Button 5


```


NOTE:  
- Charging: blinking red  
- Fully Charged: LED off  

- Low Battery: blinking red  
- Battery OK: solid red  


- Instructions say do not charge more than 2 hours  

- Power Consumption:  12 mA at 11.13v (probably 25mA at 5v through USB)`  

=== Instruction manual for 2.4 GHz Wireless USB Controller ===  

Please read before use:  
1. Make sure the controller is fully charged prior to first use to ensure battery is at maximum capacity.  
2. Make sure to use a standard 5v 1A charger to avoid damage to the controller.  
3. Do not charge the controller for over 2 hours.  
4. The red LED will flash rapidly during normal use indicating that the controller is low on battery.  
   Please charge the controller immediately to avoid losing connection.  
5. The red LED will blink slowly when the controller is charging, it will disappear once the   
   controller is fully charged and ready for use.  
6. When the controller and the receiver cannot be connected successfully,  
   press the DOWN+SELECT+START buttons at the same time, and the red LED  
   of the controller flashes to reset.  They can be paired successfully.  


Pairing the controller:  
1. Turn on the controller by pressing the START button, the red LED will blink slowly  
2. Press the START button again, the red LED will flash rapidly indicating the controller  
   is ready to pair.  
3. Plug in the USB receiver to the PC, the red LED will stay lit  
   to indicate the controller has been paired successfully.  


LED Functions:  
1.  Press START button, LED flashes slowly: controller is "On"  
2.  With controller on, press START again, and LED flashes quickly: searching for adapter  
3.  LED on continuously:  connected/paired successfully  
4.  During charging, LED flashing every 2 seconds: charging - LED turns off when fully charged  
5.  LED flashes quickly intermittently:  battery low  


# Examples

For Python access, install evdev:

```
sudo pip3 install --break-system-packages evdev

see: https://python-evdev.readthedocs.io/en/latest/usage.html

pi@RPi5DESK:~/pi5desk/systests/joy $ sudo python3 -m evdev.evtest
ID  Device               Name                                Phys                                Uniq
-----------------------------------------------------------------------------------------------------------------
0   /dev/input/event0    pwr_button                          gpio-keys/input0                        
1   /dev/input/event1    vc4-hdmi-0                          vc4-hdmi-0/input0                       
2   /dev/input/event2    vc4-hdmi-1                          vc4-hdmi-1/input0                       
3   /dev/input/event3    Jieli Technology UACDemoV1.0        usb-xhci-hcd.1-2/input2             1120040804060316
4   /dev/input/event4    Controller                          usb-xhci-hcd.0-1/input0                 

Select devices [0-4]: 4
Listening for events (press ctrl-c to exit) ...

=== PRESSED JOYPAD-UP
time 1703175481.424277 type 3 (EV_ABS), code 1    (ABS_Y), value 0
time 1703175481.424277 --------- SYN_REPORT --------
time 1703175481.576286 type 3 (EV_ABS), code 1    (ABS_Y), value 128
time 1703175481.576286 --------- SYN_REPORT --------
time 1703175482.344332 type 3 (EV_ABS), code 1    (ABS_Y), value 255
time 1703175482.344332 --------- SYN_REPORT --------
time 1703175482.520343 type 3 (EV_ABS), code 1    (ABS_Y), value 128
time 1703175482.520343 --------- SYN_REPORT --------
time 1703175483.080381 type 3 (EV_ABS), code 0    (ABS_X), value 0
time 1703175483.080381 --------- SYN_REPORT --------

=== PRESSED JOYPAD-DN
time 1628955329.737951 type 3 (EV_ABS), code 1    (ABS_Y), value 255
time 1628955329.737951 --------- SYN_REPORT --------
time 1628955329.907947 type 3 (EV_ABS), code 1    (ABS_Y), value 128
time 1628955329.907947 --------- SYN_REPORT --------

=== PRESSED JOYPAD-RIGHT
time 1628955374.335807 type 3 (EV_ABS), code 0    (ABS_X), value 255
time 1628955374.335807 --------- SYN_REPORT --------
time 1628955374.451807 type 3 (EV_ABS), code 0    (ABS_X), value 128
time 1628955374.451807 --------- SYN_REPORT --------

=== PRESSED JOYPAD-LEFT
time 1628955400.639726 type 3 (EV_ABS), code 0    (ABS_X), value 0
time 1628955400.639726 --------- SYN_REPORT --------
time 1628955400.767729 type 3 (EV_ABS), code 0    (ABS_X), value 128
time 1628955400.767729 --------- SYN_REPORT --------

```

# FOR JOYSTICK IN DOCKER SEE ../configs/docker/README
