#!/usr/bin/env python3

# FILE:  test_docking.py

"""
   USAGE:  Set NUM_OF_DOCKING_TESTS and then execute ./test_docking.py
"""


import sys
sys.path.append('/home/pi/GoPi5Go/plib')
from myeasygopigo3 import EasyGoPiGo3
from time import sleep



def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    try:
            egpg.stop()
    except KeyboardInterrupt:
        egpg.stop()
    finally:
        print("\nSTOPPED!")

if __name__ == '__main__':
    main()
