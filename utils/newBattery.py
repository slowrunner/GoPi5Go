#!/usr/bin/env python3

# FILE:  newBattery.py

# PURPOSE: Update daveData.json when battery set is changed

# VALUES AFFECTED:
#   newBatteryDate  :  2020-08-21
#   newBatteryAtDocking  :  1454
#   newBatteryAtLifeHours  :  11870
#   newBatteryDesc  :  




import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
import daveDataJson

def main():
    try:
        keys = ["newBatteryDate", "newBatteryAtDocking", "newBatteryAtLifeHours", "newBatteryDesc"]
        print("New Battery Set Util For daveData.json")

        for key in keys:
            daveDataJson.printData()
            print("\n")
            while True:
                prompt = "Enter new \"{}\" : ".format(key)
                val = input(prompt)
                if val == "":
                    val = daveDataJson.getData(key)
                prompt = "Use value \"{}\" y/n? ".format(val)
                if input(prompt) == "y": break


            if (daveDataJson.saveData(key, val) == True):
                print('   Saved {}: {}'.format(key,val))
            else:
                print("   saveDaveData({}) failed".format(key))

            print("\n")


    except KeyboardInterrupt:
        print("\nExiting")

    finally:
        daveDataJson.printData()


if __name__ == "__main__":
    main()
