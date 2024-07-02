#!/usr/bin/env python3

"""
    FILE:  spi_mutex.py

    Use to ensure only one process at a time accesses the SPI bus.

    USAGE:
        Put spi_mutex.py in execution folder of each program that uses SPI 

        or put spi_mutex.py somewhere and:

        import os
        os.path.insert(1,'/full/path/to/folder_containing_spi_mutex.py_file/')

        from spi_mutex import SPI_Mutex

        spi_mutex = SPI_Mutex()


        try:
            spi_mutex.acquire()
            # Do SPI bus operations
        finally:
            spi_mutex.release()

    NOTE:  Puts lock file SPI_Mutex.lck in /run/lock


"""

import datetime as dt
import time
import atexit
import fcntl
import os


DEBUG = False


class SPI_Mutex(object):

    def __init__(self, loop_time = 0.0001):
        """ Initialize """

        self.Filename = "/run/lock/SPI_Mutex.lck"
        self.LoopTime = loop_time
        self.Handle = None

        try:
            self.Handle = open(self.Filename, 'w')
            if os.path.isfile(self.Filename):
                os.chmod(self.Filename, 0o777)

        except Exception as e:
            print(e)

        # Register the exit method
        atexit.register(self.__exit_cleanup__) # register the exit method

    def __exit_cleanup__(self):
        """ Called at exit to clean up """

        self.release()
        self.Handle.close()

    def acquire(self):
        """ Acquire the mutex """

        while True:
            try:
                fcntl.flock(self.Handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return
            except IOError: # already locked by a different process
                if DEBUG: print("mutex locked, waiting")
                time.sleep(self.LoopTime)
            except Exception as e:
                print(e)

    def release(self):
        """ Release the mutex """

        if self.Handle is not None and self.Handle is not True:
            fcntl.flock(self.Handle, fcntl.LOCK_UN)
            time.sleep(self.LoopTime)




# EXAMPLE - Use with 2nd_SPI_user.py example

DT_FORMAT = "%Y-%m-%d %H:%M:%S.%f"

USER = "SPI_user"

def main():

    spi_mutex = SPI_Mutex()

    doit = True
    while doit:

        try:
            spi_mutex.acquire()
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"|",USER,": I got the mutex")
            time.sleep(5)
        except KeyboardInterrupt:
            print("\n")
            doit = False
        finally:
            spi_mutex.release()
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"|",USER,": I released the mutex\n")


if __name__ == '__main__':
    main()
