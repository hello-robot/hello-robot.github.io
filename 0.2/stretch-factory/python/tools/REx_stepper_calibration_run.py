#!/usr/bin/env python

from future.builtins import input
import sys
import time
import stretch_body.stepper as stepper
import stretch_body.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the encoder on a benchtop stepper')
parser.add_argument('usb', metavar='full_usb_path', type=str, nargs=1,help='Provide full usb path e.g.: /dev/hello-motor-lift')
args=parser.parse_args()

motor = stepper.Stepper(args.usb[0])
if not motor.startup():
    exit(1)

motor.push_command()
motor.turn_menu_interface_on()

time.sleep(0.5)
#motor.print_menu()

calibration_done=False


i=0
while i<3 and not calibration_done:
    print('Doing step ',i)
    motor.menu_transaction(b's')
    yn=input('Did motor step (y/n)[n]?')
    i=i+1
    if yn=='y':
        print('Starting encoder calibration')
        motor.menu_transaction(b'c')
        calibration_done=True
        input('Hit enter when calibration done...')
        break
if calibration_done:
    print('Calibration success.')
else:
    print('Calibration failure')
motor.turn_rpc_interface_on()
motor.push_command()
motor.board_reset()
motor.push_command()


