#!/usr/bin/env python3

# FILE:  undock.py

"""
   USAGE:  Execute ./undock.py to drive off dock
"""


import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
import docking

def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    try:
        docking.undock(egpg)
    except KeyboardInterrupt:
        egpg.stop()
        print("\nTest Terminated")
    finally:
        egpg.stop()

if __name__ == '__main__':
    main()
