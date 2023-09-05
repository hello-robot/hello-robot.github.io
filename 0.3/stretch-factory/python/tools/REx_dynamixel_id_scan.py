#!/usr/bin/env python

from stretch_body.dynamixel_XL430 import DynamixelXL430, DynamixelCommError
import argparse

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

import stretch_body.device
d = stretch_body.device.Device(name='dummy_device',req_params=False) # to initialize logging config

parser=argparse.ArgumentParser(description='Scan a dynamixel bus by ID for servos')
parser.add_argument("usb_full_path", help="The full path to dynamixel USB bus e.g.: /dev/hello-dynamixel-head")
args = parser.parse_args()

m=None
nfind=0
try:
    print('Scanning bus %s' % (args.usb_full_path))
    for id in range(25):
        print('Checking ID %d'%id)
        baud = DynamixelXL430.identify_baud_rate(id, args.usb_full_path)
        if baud !=-1:
            m = DynamixelXL430(id, args.usb_full_path,baud=baud)
            if not m.hw_valid:
                print('Unable to open bus %s. Exiting'%args.usb_full_path)
                exit(1)
            try:
                m.startup()
                nfind=nfind+1
            except DynamixelCommError:
                print("Ping failed for ID %d and baud %d: "%(str(id),b))
                continue
            m.do_ping()
            m.stop()
except (KeyboardInterrupt, SystemExit):
    if m is not None:
        m.stop()

print('Found %d  servos on bus %s'%(nfind,args.usb_full_path))