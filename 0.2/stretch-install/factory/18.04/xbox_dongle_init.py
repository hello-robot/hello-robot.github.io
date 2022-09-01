#!/usr/bin/env python
from __future__ import print_function
from inputs import get_gamepad

def main():
    try:
        events = get_gamepad()
    except Exception as e:
        print("Exception thrown--------------")
        print(e)
    else:
        print("dongle connected--------------")

if __name__ == "__main__":
    main()
