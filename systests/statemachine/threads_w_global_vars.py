#!/usr/bin/env python3

"""
FILE:  threads_w_global_vars.py

DOC:   Demonstrates basic threading architecture with global variables for inter-thread communication
       Global vars are not recommended practice, but can work with a thread lock for small applications.

"""
from threading import Thread,Lock

import time


def f_thread1(threadname):
    global global_a, global_b      # only reads globals so no need to use global_lock
    while True:
        print(f"{threadname}: global_a: {global_a} global_b: {global_b}")
        time.sleep(0.5)

def f_thread2(threadname):
    global global_a,global_b, global_lock
    while True:
        with global_lock:   # remember to keep globals locked very short, to not make others wait long
            global_a += 1
            global_b = global_a
        print(f"{threadname}: incremented a: {global_a} and set b=a {global_b}")
        time.sleep(1)

def f_thread3(threadname):
    global global_a,global_b, global_lock
    while True:
        with global_lock:  # keep locked to a minimum
            global_b += 1
        print(f"{threadname}: incremented b: {global_b}")
        time.sleep(1)

def main():
    global global_a, global_b, global_lock

    global_a = 0  # global var to share among threads
    global_b = 0  # global var to share among threads

    global_lock = Lock()  # use to lock all variables when changing any of them (maintain synchronization)

    thread1 = Thread( target=f_thread1, args=("Thread-1",) )
    thread2 = Thread( target=f_thread2, args=("Thread-2",) )
    thread3 = Thread( target=f_thread3, args=("Thread-3",) )

    thread1.start()
    thread2.start()
    thread3.start()

    thread1.join()
    thread2.join()
    thread3.join()


if __name__ == "__main__":  main()
