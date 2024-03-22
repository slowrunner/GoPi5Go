# Log Value And Print (GoPi5Go-Dave Version)

C++ EasyGoPiGo3 API project to monitor battery voltage, and generate plot of "life" 

Example battery till shutdown plot (Oak-D-Lite and YPLIDAR X4 powered at idle):  

<img src="pic/2024-03-22_batteryLife.png" width="60%">  

**Introduction:**  
- To make:
  cmake CMakeLists.txt
  make

- To run:
  build/batteryLife    
  build/batteryLife > batteryLife.out 2>&1 &
  
  ./plotBattLife.py  csv/YYYY-MM-DD_batteryLife.csv
 
- batteryLife:  Checks battery voltage every 6 seconds, and writes value to csv file  
- plotBattLife.py:  Creates a graphic of voltage vs up time in <date>_batteryLife.png  

- .csv files are written to      <base_folder>/csv/         (created if not existing)  
- .png plot files are written to <base_folder>/pic/         (created if not existing)  

**Hardware:**  
- Raspberry Pi 5 4GB running with PiOS Bookworm
- DexterIndustries GoPiGo3
- DexterIndustries/ModularRobotics 3000mA 11.1v Li-Ion Battery
- DexterIndustries/ModularRobotics 120v 1A Li-Ion Battery Charger
  
**DIODE_DROP:**
- DIODE_DROP while docked 0.56v
- DIODE_DROP while undocked 0.76v

Packages needed to be installed:  
- matplotlib  ```sudo pip3 install matplotlib --break-system-packages```  

  
