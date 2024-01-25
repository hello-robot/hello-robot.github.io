#!/usr/bin/env python3

import stretch_body.stepper as stepper
import stretch_body.pimu as pimu
import stretch_body.hello_utils as hu
import argparse
import time


motor_names = ['hello-motor-left-wheel', 'hello-motor-right-wheel', 'hello-motor-arm', 'hello-motor-lift']
parser = argparse.ArgumentParser(description='Read and write stepper type to/from flash memory of stepper boards')
group2 = parser.add_mutually_exclusive_group(required=True)
group2.add_argument("--read", help="Read the current stepper type from flash",action="store_true")
group2.add_argument("--write", help="Write stepper_type to stepper flash",action="store_true")
args = parser.parse_args()


def stepper_type():
    for i in motor_names:
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

