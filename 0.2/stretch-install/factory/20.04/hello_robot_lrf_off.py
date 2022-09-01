#!/usr/bin/env python3
import serial
import time
#This script keeps the lrf port open while the computer is on
#This has the effect of keeping the DTR line high to the LRF, which in turn turns the motor off
#So long as one port is open on the serail the DTR remains high (see HUPcL in termios)
#Other programs can still open/close/read/write the port independent of this script
sp=serial.Serial('/dev/hello-lrf',115200,parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1, dsrdtr=True)
while True:
    time.sleep(1.0)
