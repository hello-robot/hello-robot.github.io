#!/usr/bin/env python
from future.builtins import input
from stretch_body.dynamixel_XL430 import *
import argparse
import stretch_body.device
d = stretch_body.device.Device(name='dummy_device',req_params=False) # to initialize logging config

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()


parser=argparse.ArgumentParser(description='Set the ID of a Dynamixel servo')
parser.add_argument("usb_full_path", help="The full path to the dynamixel USB bus e.g.: /dev/hello-dynamixel-head")
parser.add_argument("id_from", help="The ID to change from", type=int)
parser.add_argument("id_to", help="The ID to change to", type=int)
args = parser.parse_args()

try:
    baud = DynamixelXL430.identify_baud_rate(args.id_from, args.usb_full_path)
except:
    print("Unable to detect baud at ID %d on bus %s"%(args.id_from,args.usb_full_path))
    exit(1)
if baud !=-1:
    m = DynamixelXL430(args.id_from, args.usb_full_path,baud=baud)
    m.startup()
    if not m.do_ping():
        exit(0)

    input('Ready to change ID %d to %d. Hit enter to continue'%(args.id_from,args.id_to))
    m.disable_torque()
    m.set_id(args.id_to)

    m = DynamixelXL430(args.id_to, args.usb_full_path,baud=baud)
    m.startup()
    if not m.do_ping():
        print('Failed to set new ID')
    else:
        print('Success at setting ID to %d'%args.id_to)
else:
    print("Unable to detect baud at ID %d on bus %s"%(args.id_from,args.usb_full_path))

