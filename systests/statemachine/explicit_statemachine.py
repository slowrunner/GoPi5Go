#!/usr/bin/env python3


"""
FILE: explicit_statemachine.py

DOC:  simple statemachine (no transitions, actions, events)

"""



import time



def initial():
    global state, states
    time.sleep(1)   # state actions
    state +=1
    print(f"initial(): transition to {states[state]}")

def state1():
    global state, states
    time.sleep(1)
    state += 1
    print(f"state1(): transition to {states[state]}")

def state2():
    global state, states
    time.sleep(1)
    state +=1
    print(f"state2(): transition to {states[state]}")

def final():
    global state, states
    time.sleep(1)
    print("final():  we're done here!")
    state +=1

def main():
    global state, states

    states = ["INITIAL","STATE1","STATE2","FINAL"]

    state = 0
    while True:
        if state == 0:  initial()
        elif state == 1: state1()
        elif state == 2: state2()
        elif state == 3: final()
        else:  exit()


if __name__ == "__main__": main()
