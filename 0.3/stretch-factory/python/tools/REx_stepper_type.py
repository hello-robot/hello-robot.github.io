#!/usr/bin/env python3

import stretch_body.stepper as stepper
import stretch_body.pimu as pimu
import stretch_body.hello_utils as hu
import argparse
import time


parser = argparse.ArgumentParser(description='Read or Write Stepper Type to Flash Memory of Stepper Boards \n eg:(REx_stepper_type.py --write --arm)')

group = parser.add_mutually_exclusive_group()
group.add_argument("--write", help="Write Stepper Type to Flash Memory", action="store_true")
group.add_argument("--read", help="Read Stepper Type from Flash Memory", action="store_true")


parser.add_argument("--arm", help="Read/Write stepper type from arm flash memory", action="store_true")
parser.add_argument("--lift", help="Read/Write stepper type from lift flash memory", action="store_true")
parser.add_argument("--left_wheel", help="Read/Write stepper type from left wheel flash memory", action="store_true")
parser.add_argument("--right_wheel", help="Read/Write stepper type from right wheel flash memory", action="store_true")

args = parser.parse_args()


def stepper_type():

    if args.arm or args.lift or args.left_wheel or args.right_wheel:
        use_device={'hello-motor-lift':args.lift,'hello-motor-arm':args.arm, 'hello-motor-right-wheel':args.right_wheel, 'hello-motor-left-wheel':args.left_wheel}
    else:
        use_device = {'hello-motor-arm': True, 'hello-motor-right-wheel': True, 'hello-motor-left-wheel': True,'hello-motor-lift': True}

    for i in use_device:
        if use_device[i]:
            motor = stepper.Stepper(f'/dev/{i}')
            for x in motor.supported_protocols.keys():
                recent_protocol = x.strip('p')
            if int(recent_protocol) >= 5:
                if not motor.startup():
                    print(f"Error with communication to {i}")
                    exit(1)
                if motor.board_info['protocol_version'] == 'p5':
                    if args.write:
                        print(f"Now setting {i} stepper type to flash...")
                        motor.write_stepper_type_to_flash(i)
                        time.sleep(1)
                        motor.read_stepper_type_from_flash()
                        if i == motor.board_info['stepper_type']:
                            print(f"Success {i} stepper type to flash\n")
                        else:
                            print(f"Error setting stepper type to {i}, please try again\n")
                        motor.stop()
                        time.sleep(1)

                    if args.read:
                        print(f"Now reading stepper_type from {i}....")
                        motor.read_stepper_type_from_flash()
                        time.sleep(1)

                        print(f"stepper_type == {motor.board_info['stepper_type']}\n")
                        time.sleep(1)
                        motor.stop()
                else:
                    print(f"Protocol version for {i} is incorrect please update firmware to proper version")
                    exit(1)
            elif int(recent_protocol) < 5:
                print("Stretch Body Protocol Version is below p5 please update Stretch Body")
                exit(1)

stepper_type()

