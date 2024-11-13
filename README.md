# GoPi5Go

**Autonomous ROS2 home robot based on GoPiGo3 and Raspberry Pi 5**

Reincarnating Humble Dave (HumbleDave) and ROSbot Dave (rosbot-on-gopigo3)  
on the Raspberry Pi5 Single Board Computer  



<img src="https://github.com/slowrunner/GoPi5Go/blob/main/Graphics/2024-03-17_Front_GoPi5Go_Dave.JPG" width="378" height="504" />


GoPi5Go-Dave Specs:

- Platform: GoPiGo3 from ModularRobotics 

- Processor: Raspberry Pi 5
  * 2.4 GHz Max
  * Quad-core 64-bit Arm Cortex-A76
  * 4GB Memory
  * Onboard Dual Band 802.11ac Wi-Fi
  * 2x USB 3.0 ports (up to 5Gbps operation)
  * 2x USB 2.0 ports

- Docker ROS 2 Container
  *  OS: Ubuntu 22.04 basic 
  *  ROS2 Humble Hawksbill
 
- Control Interfaces: 
  * ssh over WiFi
  * ROS2 Humble Hawksbill in Docker
  * 2.4GHz Wireless USB Logitech F710 Gamepad 

- Sensors (GoPiGo3 Intrinsic)
  * Battery_Voltage (GoPiGo3 intrinsic)
  * Regulated_5v_Voltage (GoPiGo3 intrinsic)
  * Magnetic Wheel Encoders 1920 cnt/rev (GoPiGo3 intrinsic, 16-tick motors)

- Sensors (Raspberry Pi Intrinsic)  
  * Processor Temperature 
  * Processor Low Voltage Throttling Active / Latched
  * Processor Temperature Throttling Active / Latched
  
- Sensors (Added):
  * YDLIDAR X4 - 360 degrees, 12cm-10m range on half degree increment, ~8Hz scanning
  * MPU9250 Inertial Measurement Unit
    also provides ambient temperature 
  * Oak-D-W-97: RGB Center camera, Stereo Grayscale Depth cameras  
    HFOV: 150 VFOV: 80 deg Resolution: 1Mpx 1280x800  
  
- Actuators/Effectors (GoPiGo3 Intrinsic)
  * Wheel Motors
  * Multi-color programmable LED (x3)
  * Program controlled Red LED (x2)
  * Tri-color Battery Voltage Indicator

- Added Actuators/Effectors 
  * USB audio speaker  
  
- Available GoPiGo3 Ports
  * I2C: MPU9250 IMU  
  * I2C: Unused  
  * Grove Analog/Digital I/O AD1: Unused  
  * Grove Analog/Digital I/O AD2: Unused   
  * SERVO1: Unused  
  * SERVO2: Unused  

- Power Source: ModRobotics 3000mAH 11.1v Rechargeable Battery  
  * 12.3v to protection circuit cutoff at 8.1-8.4v!   
  * Roughly 25wH  
  * Charger never reaches trickle charge due to powering Pololu 5v circuit  

- Play Time: (Using 10.1v 15minutes left "need to shutdown" limit)  
  * "GoPi5Go-Dave ROSbot" 2.8 hours  (averages 7.5w 20wH)  
  * "100% wandering" 2.5 hours  

- Recharger:  
  * ModRobotics Li-ion Battery Charging adapter  
  * 12.6v 1A output with charging/charged LED  
  * About 2.4 hours recharge from 10.1v docking to 100mA charging current  

- Physical:
  * 2.5 lbs Total
  * 7" wide x 9" Long x 12" High

- Total Cost: $782

- First "Life as GoPi5Go-Dave": March 2024    (PiOS Bookworm, ROS 2 Humble over Ubuntu 22.04 in Docker)  
- First "Life as Humble-Dave": Oct 7, 2022    (ROS 2 Humble, Ubuntu 22.04)  
- First "Life as ROSbot Dave": June 12, 2021  (ROS 2 Foxy, Ubuntu 20.04)  
- First "Life as ROS-GoPiGo3": Apr 2019       (Robot Carl, ROS Kinetic, Raspbian Stretch)  

- GoPiGo3 API Modifications for Bookworm/Raspberry Pi 5
  * Reduced SPI transfer rate to 250 khz for Pi5
  * Removed Software I2C for Bookworm
  * Removed all pigpio dependancy (Was configuring SPI for ALT0 - no longer needed)
  * Changed distance sensor and distance sensor examples to default to hardware I2C
  * Changed power monitor to gpiod (was RPi.GPIO)

# SETUP:  
[Setup Document](config/SETUP.md) 

# Power and Load Statistics  
# ROS2 Humble / Ubuntu 22.04 in Docker on Raspberry Pi 5 running PiOS Bookworm 64-bit Desktop 
- Undocked: 7.7W 1.0GB Load 0.26 = 5% of Pi5 CPU  
- + LIDAR: 8.4W 1.0GB Load 0.30  = 7% CPU  
- + Nav2:  8.8W  1.2GB Load 0.95 = 25% CPU  
- To Nav2 Goal:  10.4W 1.2GB Load 0.98 = 25% CPU  
- (lost) 1.3GB Load 1.20 = 30% CPU
 
# FINAL DISPOSITION
- I believe poor odometry accuracy messes up the LIDAR localization.  Don't know enough to fix it.
- After three years of working with ROS 2, I'm giving up on GoPiGo3 robot as a ROS platform
- (Create3-Wali almost met the bill, but suffered from a severely underpowered processor.)  

```
*** GoPi5Go Dave TOTAL LIFE STATISTICS ***
Total Awake:   5623.3  hrs
Total Naps:     57.05  hrs
Total Life:    5680.35  hrs (since Mar 17, 2024)
GoPi5Go-Dave Playtimes (Undocked-Docked): 878
Total Dockings:  1276
New Battery Installed At Docking: 416
This Battery At Cycle:  860
Average playtime (last three) 2.6 hrs 
Average docked time (last three) 2.6 hrs 
Sessions (boot):  93
Average Session:  60.4 hrs
Safety Shutdowns:  20
Total Travel:  1134.0 meters 3720.4 feet
 
Last Undocking String:  2024-11-12 20:25|dave_node.py| ---- GoPi5Go-Dave ROS 2 Undocking, Charge Current 99 mA 12.1v after 2.6 h charging
Last Docking   String:  2024-11-12 17:49|dave_node.py| ---- GoPi5Go-Dave ROS 2 Docking 1276 : success at battery 10.1v after 2.7 h playtime +

********** ROS2 GoPiGo3 Status ******************************
Tuesday 11/12/24
 22:06:54 up 31 days,  3:19,  3 users,  load average: 0.17, 0.30, 0.34
temp=45.0'C
frequency(0)=1500019456
throttled=0x0
Current Battery 10.55v  37.8% Load: 724mA 7.6W
               total        used        free      shared  buff/cache   available
Mem:           4.0Gi       1.1Gi       685Mi        13Mi       2.3Gi       2.9Gi
Swap:          199Mi       145Mi        54Mi
GoPiGo3 Battery Voltage: 10.6 volts


ROS 2 NODES
pi       2246409 2246387  0 15:08 pts/0    00:00:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 run ros2_gopigo3_node gopigo3_node
pi       2246468 2246387  0 15:08 pts/0    00:00:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 run gopi5go_dave battery_node
pi       2246500 2246387  0 15:08 pts/0    00:00:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 run gopi5go_dave docking_node
pi       2246514 2246387  0 15:08 pts/0    00:00:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 run ros2_gopigo3_node odometer
pi       2246553 2246387  0 15:08 pts/0    00:00:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 run gopi5go_dave say_node
pi       2246617 2246387  0 15:08 pts/0    00:00:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 run gopi5go_dave dave_node
pi       2246433 2246387  0 15:08 pts/0    00:00:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 launch ros2_gopigo3_node ros2_gopi5go_dave_state_and_joint.launch.py
pi       2246576 2246387  0 15:08 pts/0    00:00:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 launch teleop_twist_joy teleop-launch.py joy_config:=F710


```

