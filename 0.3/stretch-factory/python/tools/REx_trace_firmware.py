#!/usr/bin/env python
import argparse
import stretch_body.pimu
import stretch_body.wacc
import stretch_body.stepper
import stretch_body.hello_utils as hu
from stretch_factory.trace_mgmt import TraceMgmt

hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='Tool to load and view the firmware trace data.', )
group = parser.add_mutually_exclusive_group()
group.add_argument("--pimu", help="Trace Pimu firmware", action="store_true")
group.add_argument("--wacc", help="Trace Wacc firmware", action="store_true")
group.add_argument("--arm", help="Trace Arm Stepper firmware", action="store_true")
group.add_argument("--lift", help="Trace Lift Stepper firmware", action="store_true")
group.add_argument("--left_wheel", help="Trace Left Wheel Stepper firmware", action="store_true")
group.add_argument("--right_wheel", help="Trace Right Wheel Stepper firmware", action="store_true")
args = parser.parse_args()

if not (args.pimu or args.wacc or args.arm or args.lift or args.left_wheel or args.right_wheel):
    print('Device argument required')
    exit(0)

if args.pimu:
    device=stretch_body.pimu.Pimu()
if args.wacc:
    device=stretch_body.wacc.Wacc()
if args.arm:
    device=stretch_body.stepper.Stepper('/dev/hello-motor-arm')
if args.lift:
    device=stretch_body.stepper.Stepper('/dev/hello-motor-lift')
if args.left_wheel:
    device=stretch_body.stepper.Stepper('/dev/hello-motor-left-wheel')
if args.right_wheel:
    device=stretch_body.stepper.Stepper('/dev/hello-motor-right-wheel')


device.startup()

mgmt=TraceMgmt(device)
mgmt.run_menu()