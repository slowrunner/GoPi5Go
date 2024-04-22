#!/usr/bin/env python3

import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
import daveDataJson



try:
    chargeCycles = int(daveDataJson.getData('chargeCycles'))
    chargeCycles += 1
except:
    chargeCycles = 0


if (daveDataJson.saveData('chargeCycles', chargeCycles) == True):
        print('   Saved chargeCycles: {}'.format(chargeCycles))
else:
        print("   saveData('chargeCycles') failed")

