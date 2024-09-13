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
  * "100% wandering" TBD hours  

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

