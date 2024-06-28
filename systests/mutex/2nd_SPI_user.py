#!/usr/bin/env python3

"""
    FILE:  2nd_SPI_user.py


    Example to test SPI_Mutex class

    Terminal 1: ./spi_mutex.py
    Terminal 2: ./2nd_SPI_user.py

    spi_mutex.py will hold mutex for 5 seconds
    2nd_SPI_user.py will hold mutex for 1 second

"""

import datetime as dt
import time
from spi_mutex import SPI_Mutex

DT_FORMAT = "%Y-%m-%d %H:%M:%S.%f"

USER = "2nd_SPI_user"

def main():

    spi_mutex = SPI_Mutex()


    while True:

        try:
            spi_mutex.acquire()
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"|",USER,": I got the mutex")
            time.sleep(1)
        finally:
            spi_mutex.release()
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"|",USER,": I released the mutex\n")


if __name__ == '__main__':
    main()
