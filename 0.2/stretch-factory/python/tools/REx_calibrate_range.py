#!/usr/bin/env python

import stretch_body.dynamixel_hello_XL430 as dxl
from stretch_body.hello_utils import *

import argparse
import click

print_stretch_re_use()

parser = argparse.ArgumentParser(description='Calibrate the range of motion for specified joint.')
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--head_pan", help="Calibrate the head_pan joint", action="store_true")
group.add_argument("--head_tilt", help="Calibrate the head_tilt joint", action="store_true")
group.add_argument("--wrist_yaw", help="Calibrate the wrist_yaw joint", action="store_true")
group.add_argument("--arm", help="Calibrate the arm joint", action="store_true")
group.add_argument("--lift", help="Calibrate the lift joint", action="store_true")
args = parser.parse_args()

if args.head_pan or args.head_tilt or args.wrist_yaw:
    if args.head_pan:
        h=dxl.DynamixelHelloXL430('head_pan')
    if args.head_tilt:
        h = dxl.DynamixelHelloXL430('head_tilt')
    if args.wrist_yaw:
        h = dxl.DynamixelHelloXL430('wrist_yaw')
    h.params['req_calibration']=1
    if not h.startup():
        exit(1)
    h.home(single_stop=False,move_to_zero=True,delay_at_stop=1.0,save_calibration=True, set_homing_offset=False)
    print('Recalibration done.')
    h.stop()

if args.arm or args.lift:
    if args.arm:
        import stretch_body.arm
        j= stretch_body.arm.Arm()
    if args.lift:
        import stretch_body.lift
        j= stretch_body.lift.Lift()
    if not j.startup(threaded=False):
        exit(1)
    j.pull_status()
    if not j.motor.status['pos_calibrated']:
        print('Joint not calibrated. Exiting.')
        j.stop()
        exit(1)

    if (j.name in j.user_params and 'range_m' in j.user_params[j.name]):
        click.secho('------------------------', fg="yellow")
        click.secho('NOTE: This tool updates range_m for %s in stretch_configuration_params.yaml' % j.name.upper(),fg="yellow")
        click.secho('NOTE: Your stretch_user_params.yaml overrides range_m for %s' % j.name.upper(),fg="yellow")
        click.secho('NOTE: As such, the updated calibration will not change the behavior unless you remove the user params.',fg="yellow")
    click.secho('------------------------', fg="yellow")
    click.secho('Joint %s will go through its full range-of-motion. Ensure workspace is collision free ' % j.name.capitalize(),fg="yellow")
    if click.confirm('Proceed?'):
        measured_rom = j.home(measuring=True)
        if measured_rom is not None:
            if (measured_rom>=j.params['calibration_range_bounds'][0] and measured_rom<=j.params['calibration_range_bounds'][1]):
                click.secho('%s measured range-of-motion: %f . Within acceptable range of %f to %f' %(j.name.capitalize(), measured_rom,j.params['calibration_range_bounds'][0], j.params['calibration_range_bounds'][1]),fg='green')
                if click.confirm('Save calibrated range of motion?'):
                    j.write_configuration_param_to_YAML('%s.range_m'%j.name, [0,measured_rom])
            else:
                click.secho('%s measured range-of-motion: %f . Outside acceptable range of %f to %f' %
                           (j.name.capitalize(), measured_rom, j.params['calibration_range_bounds'][0],j.params['calibration_range_bounds'][1]), fg='yellow')

    j.stop()



