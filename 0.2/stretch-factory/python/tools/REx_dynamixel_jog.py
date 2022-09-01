#!/usr/bin/env python

from future.builtins import input
import sys
from stretch_body.dynamixel_XL430 import *
import argparse
import stretch_body.device
d = stretch_body.device.Device(name='dummy_device',req_params=False) # to initialize logging config
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog a Dynamixel servo from the command line')
parser.add_argument("usb_full_path", help="The full path to the dynamixel USB bus e.g.: /dev/hello-dynamixel-head")
parser.add_argument("id", help="The ID to jog", type=int)

args = parser.parse_args()

baud = DynamixelXL430.identify_baud_rate(dxl_id=args.id, usb=args.usb_full_path)
if baud==-1:
    print('Unable to identify servo. Exiting')
    exit(1)

m = DynamixelXL430(args.id, args.usb_full_path,baud=baud)
if not m.startup() or not m.do_ping():
    print('Failed to start servo with given usb/id/baud info')
    exit(1)

m.disable_torque()
#If servo somehow has wrong drive mode it may appear to not respond to vel/accel profiles
#Can reset it here by uncommenting
#m.set_drive_mode(vel_based=True,reverse=False)
m.enable_pos()
m.enable_torque()



def menu_top():
    print('------ MENU -------')
    print('m: menu')
    print('a: increment position 50 tick')
    print('b: decrement position 50 tick')
    print('A: increment position 500 ticks')
    print('B: decrement position 500 ticks')
    print('v: set profile velocity')
    print('u: set profile acceleration')
    print('z: zero position')
    print('h: show homing offset')
    print('o: zero homing offset')
    print('s: set homing offset')
    print('q: got to position')
    print('p: ping')
    print('r: reboot')
    print('w: set max pwm')
    print('t: set max temp')
    print('i: set id')
    print('d: disable torque')
    print('e: enable torque')
    print('-------------------')

def step_interaction():
    menu_top()
    x=sys.stdin.readline()
    if len(x)>1:
        if x[0]=='m':
            menu_top()
        if x[0]=='d':
            m.disable_torque()
        if x[0]=='e':
            m.enable_torque()
        if x[0]=='a':
            m.go_to_pos(m.get_pos()+50)
        if x[0]=='b':
            m.go_to_pos(m.get_pos()-50)
        if x[0]=='A':
            m.go_to_pos(m.get_pos()+500)
        if x[0]=='B':
            m.go_to_pos(m.get_pos()-500)
        if x[0]=='z':
            m.disable_torque()
            m.zero_position(verbose=True)
            m.enable_torque()
        if x[0]=='h':
            m.disable_torque()
            xn=m.get_homing_offset()
            print('Current homing offset is:',xn)
        if x[0]=='s':
            m.disable_torque()
            xn = m.get_homing_offset()
            print('Current homing offset is:', xn)

            m.set_homing_offset(int(x[2:]))
            xn = m.get_homing_offset()
            print('New homing offset is:',xn)
        if x[0]=='o':
            m.disable_torque()
            xn=m.get_homing_offset()
            print('Current homing offset is:',xn)
            m.set_homing_offset(0)
            print('Homing offset set to zero')
            m.enable_torque()
        if x[0]=='v':
            v=int(x[2:])
            m.set_profile_velocity(v)
        if x[0]=='u':
            a=int(x[2:])
            m.set_profile_acceleration(a)
        if x[0]=='q':
            ff = int(x[2:])
            m.go_to_pos(ff)
        if x[0]=='t':
            tt = int(x[2:])
            m.disable_torque()
            m.set_temperature_limit(tt)
            m.enable_torque()
        if x[0]=='w':
            pp = int(x[2:])
            m.disable_torque()
            m.set_pwm_limit(pp)
            m.enable_torque()
        if x[0] == 'i':
            input('You will need to exit program and restart with new ID. Hit enter to continue')
            q = int(x[2:])
            m.disable_torque()
            m.set_id(q)
            m.enable_torque()
            print('New id is',m.get_id())
        if x[0] == 'p':
            if m.do_ping():
                print('Ping success')
            else:
                print('Ping fail')
        if x[0] == 'r':
            m.do_reboot()
    else:
        m.pretty_print()

try:
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit):
    m.disable_torque()
