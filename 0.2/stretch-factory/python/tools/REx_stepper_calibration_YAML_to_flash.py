#!/usr/bin/env python

import stretch_body.stepper as stepper
import stretch_body.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Push encoder calibration from YAML to stepper flash memory')
parser.add_argument('stepper_name', metavar='stepper_name', type=str, nargs=1,help='Provide the stepper name e.g.: hello-motor-lift')
args=parser.parse_args()

motor = stepper.Stepper('/dev/'+args.stepper_name[0])
if not motor.startup():
    exit(1)

motor.write_gains_to_flash()
motor.push_command()
print('Gains written to flash')

print('Reading calibration data from YAML...')
data=motor.read_encoder_calibration_from_YAML()
print('Writing calibration data to flash...')
motor.write_encoder_calibration_to_flash(data)
print('Successful write of FLASH. Resetting board now.')
motor.board_reset()
motor.push_command()

