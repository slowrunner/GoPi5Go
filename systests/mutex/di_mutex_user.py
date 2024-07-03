#!/usr/bin/env python3

"""
    FILE:  di_mutex_user.py


    Example to test the DI_Mutex class from the GoPiGo3 API
    - Built from RFR_Tools/miscellaneous/di_mutex.py
    - Found in RFR_Tools egg

    Test Usage:
    Terminal 1: ./di_mutex_user.py
    Terminal 2: ./d2_mutex_user.py

    di_mutex_user.py will hold mutex for 5 seconds
    d2_mutex_user.py will hold mutex for 1 second

"""

import datetime as dt
import time

from di_mutex import DI_Mutex

DT_FORMAT = "%Y-%m-%d %H:%M:%S.%f"

def main():

    spi_mutex = DI_Mutex(name="SPI")

    doit = True
    while doit:

        try:
            spi_mutex.acquire()
            # This is where code goes that uses the SPI bus

            # Simulated stuff to do while have the SPI mutex
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"|", __file__, ": I got the mutex")
            time.sleep(5)

        except KeyboardInterrupt:
            print("\n")
            doit = False
        finally:
            spi_mutex.release()
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"|", __file__, ": I released the mutex\n")


if __name__ == '__main__':
    main()
