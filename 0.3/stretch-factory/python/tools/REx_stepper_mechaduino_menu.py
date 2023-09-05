#!/usr/bin/env python
from __future__ import print_function

import sys
import stretch_body.stepper as stepper
import time
import argparse
import stretch_body.hello_utils as hu


hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Use the Mechanduino menu interface to the stepper (advanced users)')
parser.add_argument('usb', metavar='full_usb_path', type=str, nargs=1,help='Provide full usb path e.g.: /dev/hello-motor-lift')
args=parser.parse_args()


motor = stepper.Stepper(args.usb[0])
if not motor.startup():
    exit(1)

motor.push_command()
motor.turn_menu_interface_on()
time.sleep(0.5)
motor.print_menu()

try:
    while True:
        print('Menu Command>')
        s = str(sys.stdin.readline())
        motor.menu_transaction(s)
except (KeyboardInterrupt, SystemExit):
    motor.turn_rpc_interface_on()
    motor.stop()

