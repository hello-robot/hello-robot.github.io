#!/usr/bin/python3
import argparse
from stretch_factory.device_mgmt import StretchDeviceMgmt
import stretch_body.hello_utils as hu
import subprocess
import os
import sys


if os.geteuid() == 0:
    hu.print_stretch_re_use()
    parser = argparse.ArgumentParser(description='Software reset of Stretch USB devices (all by default)')
    parser.add_argument("--hello-motor-lift", help="Reset Lift USB", action="store_true")
    parser.add_argument("--hello-motor-right-wheel", help="Reset Right Wheel USB", action="store_true")
    parser.add_argument("--hello-motor-left-wheel", help="Reset Left Wheel USB", action="store_true")
    parser.add_argument("--hello-motor-arm", help="Reset Arm USB", action="store_true")
    parser.add_argument("--hello-pimu", help="Reset Pimu USB", action="store_true")
    parser.add_argument("--hello-wacc", help="Reset Wacc USB", action="store_true")
    parser.add_argument("--hello-dynamixel-wrist", help="Reset Wrist USB", action="store_true")
    parser.add_argument("--hello-dynamixel-head", help="Reset Head USB", action="store_true")

    args = parser.parse_args()
    if  any([args.hello_motor_lift, args.hello_motor_arm, args.hello_motor_left_wheel,args.hello_motor_right_wheel,args.hello_pimu,args.hello_wacc,args.hello_dynamixel_head,args.hello_wacc,args.hello_dynamixel_wrist]):
        devices_names = []
        if args.hello_motor_lift:
            devices_names.append('hello-motor-lift')
        if args.hello_motor_arm:
            devices_names.append('hello-motor-arm')
        if args.hello_motor_left_wheel:
            devices_names.append('hello-motor-left-wheel')
        if args.hello_motor_right_wheel:
            devices_names.append('hello-motor-right-wheel')
        if args.hello_pimu:
            devices_names.append('hello-pimu')
        if args.hello_wacc:
            devices_names.append('hello-wacc')
        if args.hello_dynamixel_head:
            devices_names.append('hello-dynamixel-head')
        if args.hello_dynamixel_wrist:
            devices_names.append('hello-dynamixel-wrist')
        s = StretchDeviceMgmt(device_names=devices_names)
    else:
        s=StretchDeviceMgmt()
    s.reset_all()
else:
    if sys.version_info[0]==3:
       subprocess.call(['sudo', '-E','python3'] + sys.argv)
    else:
       subprocess.call(['sudo', '-E','python'] + sys.argv)
