#!/usr/bin/env python

import stretch_body.stepper as stepper
import stretch_body.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Pull encoder calibration from stepper flash and write to YAML')
parser.add_argument('stepper_name', metavar='stepper_name', type=str, nargs=1,help='Provide the stepper name e.g.: hello-motor-lift')
args=parser.parse_args()

motor = stepper.Stepper('/dev/'+args.stepper_name[0])
if not motor.startup():
    exit(1)

data = motor.read_encoder_calibration_from_flash()
print('Read data of len',len(data))
print('Writing calibration data to YAML...')
motor.write_encoder_calibration_to_YAML(data)




