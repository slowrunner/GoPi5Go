#!/usr/bin/env python3

# file:  daveDataJson.py
#
# Serialize data values to /home/pi/GoPi5Go/daveData.json
# (running ./daveDataJson.py will create the file)
#
# Methods:
#    saveData(dataname, datavalue, logit=False)   # adds datanaem:datavalue to rosbotData.json file
#    getData(dataname=None)      # either returns dictionary with all values, or just value of passed name
#    delData(dataname)           # delete item from rosbotData.json
#    printData()                 # prints contents of rosbotData.json
#

"""
Example from Carl:
   lastDocking  :  ---- Docking 4573 completed  at 8.1 v after 2.6 h playtime
   chargeCycles  :  4573
   lastDismount  :  ---- Dismount 4573 at 10.9 v after 2.5 h recharge
   dockingState  :  1
   chargingState  :  1
   chargeConditioning  :  0
   lastDismountTime  :  2024-04-15 06:47:03
   lastRechargeDuration  :  2.5
   newBatterySetDate  :  2023-03-21
   newBatterySetAtDocking  :  3652
   newBatterySetAtLifeHours  :  36810.9
   newBatterySetDesc  :  8x Eneloop White 2000 mAh NiMH AA cells
   lastDockingTime  :  2024-04-15 04:15:21
   lastPlaytimeDuration  :  2.6

"""
import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')

import json
import threading
# import runLog
DATA_FILE = '/home/pi/GoPi5Go/daveData.json'

DataLock = threading.Lock()       # with DataLock: any operation to make syncronous

def saveData(dataname, datavalue, logit=False):


    # print("-- saveData({},{}) called".format(dataname, datavalue))
    with DataLock:         # prevents two different saveData() at same time
        lData = {}
        # print("got lock")
        try:

            lData = getData()   # lock assures no one changes this till we are done
            if lData == None:
                lData = {}
            # print("   Data:", lData)
            lData[dataname] = datavalue
            # print("   lData:",lData)

            with open(DATA_FILE, 'w') as outfile:
                json.dump( lData, outfile )
            # print("   Data.json updated")
            if logit: runLog.logger.info("** Data '{}' = {} updated **".format(dataname, datavalue))
        except:
            # print("   saveData failed")
            return False

        return True

def delData(dataname):
    # print("-- delData({}) called".format(dataname))

    with DataLock:
        lData = {}
        try:

            lData = getData()
            if lData == None:
               lData = ()
            # print("   Data:", lData)
            if dataname in lData: 
                del lData[dataname]
                # print("   lData:", lData)

                with open(DATA_FILE, 'w') as outfile:
                    json.dump( lData, outfile )
                # print("   Data.json updated")
            # else:   print("   {} not found in Data".format(dataname))
        except:
            # print("   delData{} failed".dataname)
            return False

        return True


def getData(dataname=None):


    # print("-- getData({}) called".format(dataname))

    try:
        with open(DATA_FILE, 'r') as infile:
            lData = json.load(infile)
            if (dataname == None):
                return lData
            else:
                return lData[dataname]
    except:
        # print("   getData() exception")
        return None

def printData():
    print("daveData.json contents:")
    lData = getData()
    if lData != None:
        for i in lData:
            print("  ",i," : ",lData[i])
    else:
        print("daveData.json contains no data")

def main():

    print("** Starting main()")

    printData()

    lData = getData()
    print("    daveData: ",lData)

    chargeCycles = 1

    if (saveData('chargeCycles', chargeCycles) == True):
        print('   Saved chargeCycles: {}'.format(chargeCycles))
    else:
        print("   saveData('chargeCycles') failed")

    lData = getData()
    print("    daveData: ",lData)


    chargeCycles = int(getData('chargeCycles'))
    chargeCycles += 1

    if (saveData('chargeCycles', chargeCycles) == True):
        print('   Saved chargeCycles: {}'.format(chargeCycles))
    else:
        print("   saveData('chargeCycles') failed")


    if 'chargeCycles' in lData:
        print("removing chargeCycles from daveData.json")
        delData('chargeCycles')

    if (saveData('nothing',"not important" ) == True):
        print('   Saved nothing: {}'.format("not important"))
    else:
        print("   saveData('nothing') failed")

    lData = getData()
    print("   Data: ",lData)


if __name__ == "__main__": 
    main()

