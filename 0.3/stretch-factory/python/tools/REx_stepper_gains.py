#!/usr/bin/env python

import stretch_body.stepper as stepper
import stretch_body.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Manage gains stored in stepper board flash')
parser.add_argument('stepper_name', metavar='stepper_name', type=str, nargs=1,help='Provide the stepper name e.g.: hello-motor-lift')

group2 = parser.add_mutually_exclusive_group(required=True)
group2.add_argument("--read", help="Read the current gains from flash and print to console",action="store_true")
group2.add_argument("--write", help="Write the current gains in YAML to stepper flash",action="store_true")

args=parser.parse_args()

motor = stepper.Stepper('/dev/'+args.stepper_name[0])
if not motor.startup():
    exit(1)

if args.write:
    motor.write_gains_to_flash()
    motor.push_command()
    print('Gains written to flash')

if args.read:
    motor.read_gains_from_flash()
    motor.push_command()
    motor.pull_status()
    print('-------- Gains in Flash for %s -----------'%motor.name.capitalize())
    for k in motor.gains_flash.keys():
        print('{0}: {1}'.format(k,motor.gains_flash[k]))

motor.stop()