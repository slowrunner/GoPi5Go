#!/bin/bash

# REQUIRES:
#  sudo apt install stress


# 0x50000  means throttling occurred, under-voltage occurred  
# 0x50005  means throttled now and under-voltage now  
# 0x80008  means soft temperature limit exceeded (no throttling yet)  

# RESULTS HumbleDave Pi4 2GB (Temp Throttling at 1:30, Voltage Throttling at 2:30)

# START:
# ********** ROS2 GoPiGo3 MONITOR ******************************
# Saturday 12/02/23
#  11:16:24 up 3 days, 11:30,  4 users,  load average: 0.48, 0.23, 0.17
# temp=55.5'C
# frequency(48)=1300324224
# throttled=0x0
#                total        used        free      shared  buff/cache   available
# Mem:           1.8Gi       265Mi       740Mi        15Mi       838Mi       1.4Gi
# Swap:          1.0Gi          0B       1.0Gi
# GoPiGo3 Battery Voltage: 12.5 volts

# TEMPERATURE THROTTLED:
# ********** ROS2 GoPiGo3 MONITOR ******************************
# Saturday 12/02/23
#  11:17:51 up 3 days, 11:32,  4 users,  load average: 3.87, 1.43, 0.60
# temp=81.8'C
# frequency(48)=1800404352
# throttled=0x80000     <---- TEMPERATURE THROTTLING FIRST SEEN
#                total        used        free      shared  buff/cache   available
# Mem:           1.8Gi       251Mi       755Mi        15Mi       838Mi       1.4Gi
# Swap:          1.0Gi          0B       1.0Gi
# GoPiGo3 Battery Voltage: 12.4 volts

# VOLTAGE THROTTLING OCCURRED:
# ********** ROS2 GoPiGo3 MONITOR ******************************
# Saturday 12/02/23
#  11:18:57 up 3 days, 11:33,  4 users,  load average: 4.13, 1.98, 0.85
# temp=82.7'C
# frequency(48)=1677427712
# throttled=0xe0008     <--- VOLTAGE THROTTLING OCCURRED and TEMPERATURE THROTTLED ATM
#                total        used        free      shared  buff/cache   available
# Mem:           1.8Gi       256Mi       750Mi        15Mi       838Mi       1.4Gi
# Swap:          1.0Gi          0B       1.0Gi
# GoPiGo3 Battery Voltage: 12.4 volts

# FINISH:
# ********** ROS2 GoPiGo3 MONITOR ******************************
# Saturday 12/02/23
#  11:21:43 up 3 days, 11:35,  4 users,  load average: 4.20, 2.93, 1.40
# temp=80.3'C
# frequency(48)=1300324224
# throttled=0xe0000                 <<-- BOTH VOLTAGE AND TEMPERATURE THROTTLING OCCURRED
#                total        used        free      shared  buff/cache   available
# Mem:           1.8Gi       253Mi       753Mi        15Mi       838Mi       1.4Gi
# Swap:          1.0Gi          0B       1.0Gi
# GoPiGo3 Battery Voltage: 12.5 volts


# stress four cpu cores for 5 minutes
stress -c 4 -t 300
