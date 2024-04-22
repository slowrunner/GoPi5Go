#!/usr/bin/env  python3

import sys
sys.path.insert(1,"/home/pi/GoPi5Go/plib")

import daveDataJson

# print("daveDataJson contents:")
# lcarlData = carlDataJson.getCarlData()
# for i in lcarlData:
#     print("  ",i," : ",lcarlData[i])

daveDataJson.printData()
