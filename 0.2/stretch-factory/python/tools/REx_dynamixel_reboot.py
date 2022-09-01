#!/usr/bin/env python
from stretch_body.dynamixel_XL430 import *
from stretch_body.hello_utils import *
import argparse

import stretch_body.device
d = stretch_body.device.Device(name='dummy_device',req_params=False) # to initialize logging config


import stretch_body.hello_utils as hu

hu.print_stretch_re_use()



parser=argparse.ArgumentParser(description='Reboot all of the Dynamixel servos on a bus')
parser.add_argument("usb_full_path", help="The full path to the dynamixel USB bus e.g.: /dev/hello-dynamixel-head")

args = parser.parse_args()

m=None
num_reboots=0


try:
    print('Scanning bus...')
    for id in range(25):
        print('ID %d'%id)
        baud = DynamixelXL430.identify_baud_rate(id, args.usb_full_path)
        if baud != -1:
            m = DynamixelXL430(id, args.usb_full_path,baud=baud)
            #m.startup() #Don't startup as may be in error state
            if (m.do_ping(verbose=False)):
                print('Found device %d on bus %s'%(id,args.usb_full_path))
                m.do_reboot()
                num_reboots=num_reboots+1
            else:
                m.stop()
    if num_reboots==0:
        print('Unable to detect Dynamixel devices on bus %s for reboot.'%args.usb_full_path)
    else:
        print('Rebooted %d devices'%num_reboots)
except (KeyboardInterrupt, SystemExit):
    if m is not None:
        m.stop()
