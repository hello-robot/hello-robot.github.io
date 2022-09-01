#!/usr/bin/env python

from stretch_body.dynamixel_XL430 import DynamixelXL430
import argparse
import stretch_body.device
import stretch_body.hello_utils as hu

hu.print_stretch_re_use()

d = stretch_body.device.Device(name='dummy_device',req_params=False) # to initialize logging config
parser=argparse.ArgumentParser(description='Change the baudrate of a servo')
parser.add_argument("usb", help="The dynamixel USB bus e.g.: /dev/hello-dynamixel-head")
parser.add_argument("id", help="The servo ID ", type=int)
parser.add_argument("baud", help="The new baud rate (e.g. 57600, 115200, or 1000000) for dxl", type=int, choices=[57600,115200,1000000])
args = parser.parse_args()

curr_baud = DynamixelXL430.identify_baud_rate(dxl_id=args.id, usb=args.usb)
m = DynamixelXL430(dxl_id=args.id, usb=args.usb, baud=curr_baud)
if not m.startup():
    exit(1)
change_succeeded = m.set_baud_rate(args.baud)
m.stop()

n = DynamixelXL430(dxl_id=args.id, usb=args.usb, baud=args.baud)
if change_succeeded and n.startup() and n.do_ping(verbose=False):
    print("Success at changing baud. Current baud is %d for servo %d on bus %s"%(args.baud,args.id,args.usb))
else:
    print("Failed to change baud")
