#!/usr/bin/env python3

# FILE:  mystop.py

"""
   USAGE:  Execute ./mystop.py to stop runaway GoPi5Go-Dave
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
