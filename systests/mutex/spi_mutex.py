#!/usr/bin/env python3


import datetime as dt
import time
import atexit
import fcntl
import os

class SPI_Mutex(object):
    """ Stolen from Dexter Industries mutex """

    def __init__(self, loop_time = 0.0001):
        """ Initialize """

        # self.Filename = "/var/lock/SPI_Mutex.lock"
        self.Filename = "./SPI_Mutex.lck"
        self.LoopTime = loop_time
        self.Handle = None

        try:
            self.Handle = open(self.Filename, 'w')
            if os.path.isfile(self.Filename):
                os.chmod(self.Filename, 0o666)

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
                # self.Handle = open(self.Filename, 'w')
                # lock
                # fcntl.lockf(self.Handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                fcntl.flock(self.Handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return
            except IOError: # already locked by a different process
                print("mutex locked, waiting")
                time.sleep(self.LoopTime)
            except Exception as e:
                print(e)

    def release(self):
        """ Release the mutex """

        if self.Handle is not None and self.Handle is not True:
            # fcntl.lockf(self.Handle, fcntl.LOCK_UN)
            fcntl.flock(self.Handle, fcntl.LOCK_UN)
            # self.Handle.close()
            # self.Handle = None
            time.sleep(self.LoopTime)




DT_FORMAT = "%Y-%m-%d %H:%M:%S.%f"

USER = "SPI_user"

def main():

    spi_mutex = SPI_Mutex()


    while True:

        try:
            spi_mutex.acquire()
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"|",USER,": I got the mutex")
            time.sleep(5)
        finally:
            spi_mutex.release()
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"|",USER,": I released the mutex\n")


if __name__ == '__main__':
    main()
