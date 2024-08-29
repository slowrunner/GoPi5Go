#!/usr/bin/env  python3

import sys
sys.path.insert(1,"/home/pi/GoPi5Go/plib")

import daveDataJson

# print("daveDataJson contents:")
# lcarlData = carlDataJson.getCarlData()
# for i in lcarlData:
#     print("  ",i," : ",lcarlData[i])

daveDataJson.printData()
ldaveData = daveDataJson.getData()
for key in ldaveData:
    value = ldaveData[key]
    print("\nkey:",key,": ",value)
    prompt = "Keep value \"{}\" y/n? ".format(value)
    if input(prompt) == "n": 
        while True:
            val = input("Enter New Value (without quotes): ")
            prompt = "Use value \"{}\" y/n? ".format(val)
            if input(prompt) == "y": break


        if (daveDataJson.saveData(key, val) == True):
            print('   Saved {}: {}'.format(key,val))
        else:
            print("   saveData({}) failed".format(key))

print("\n")
daveDataJson.printData()
