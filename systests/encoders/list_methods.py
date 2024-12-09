#!/bin/env python3

import easygopigo3
import gopigo3
import inspect

def list_class_members(module):
    for name, obj in inspect.getmembers(module):
        if inspect.isclass(obj):
            print(f"Class: {name}")
            for member_name, member_obj in inspect.getmembers(obj):
                print(f"  - {member_name}")

print("easygopigo3 module methods:")
list_class_members(easygopigo3)


print("\ngopigo3 module methods:")
list_class_members(gopigo3)
