#!/usr/bin/env python

import os.path
import sys
import argparse
import time
import click
from colorama import Fore, Style
from stretch_factory.firmware_available import FirmwareAvailable
import stretch_factory.hello_device_utils as hdu
import stretch_factory.firmware_utils as fwu
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='Tool to directly flash Stretch firmware to a ttyACM device', )
group = parser.add_mutually_exclusive_group()
group.add_argument("--map", help="Print mapping from ttyACMx to Hello devices", action="store_true")
group.add_argument('--flash', nargs=2, type=str, help='Flash firmware. E.g, --flash /dev/ttyACM0 hello_stepper')
group.add_argument('--boot', nargs=1, type=str, help='Place board in bootloader mode. E.g, --boot /dev/ttyACM0')
args = parser.parse_args()

if args.boot:
    hdu.place_arduino_in_bootloader(args.boot[0])

if args.map:
    mapping = hdu.get_hello_ttyACMx_mapping()
    click.secho('------------------------------------------', fg="yellow", bold=True)
    for k in mapping['hello']:
        print('%s | %s' % (k, mapping['hello'][k]))
    click.secho('------------------------------------------', fg="yellow", bold=True)
    for k in mapping['ACMx']:
        print('%s | %s' % (k, mapping['ACMx'][k]))
    click.secho('------------------------------------------', fg="yellow", bold=True)
    print('')

if args.flash:
    port = args.flash[0]
    sketch_name = args.flash[1]

    # get latest firmware
    sketch_to_device = {'hello_stepper': 'hello-motor-arm', 'hello_pimu': 'hello-pimu', 'hello_wacc': 'hello-wacc'}
    a = FirmwareAvailable({sketch_to_device[sketch_name]: True})
    version = a.get_most_recent_version(sketch_to_device[sketch_name], None).to_string()
    os.system(f"cd {a.repo_path}; git pull > /dev/null 2>&1; git checkout tags/{version} > /dev/null 2>&1")

    # verify arduino cli setup
    if not fwu.check_arduino_cli_install():
        print(Fore.RED + "Arduino CLI not available." + Style.RESET_ALL)
        sys.exit(1)
    acli_path = hdu.create_arduinocli_config_file(a.repo_path)

    # compile firmware
    if not hdu.compile_arduino_firmware(sketch_name, a.repo_path, config_file=acli_path):
        print(Fore.RED + f"Failed to compile Arduino Sketch:{sketch_name}." + Style.RESET_ALL)
        sys.exit(1)
    print(Fore.GREEN + f"Compiled Arduino Sketch:{sketch_name} Successfully." + Style.RESET_ALL)
    # hdu.place_arduino_in_bootloader(port)
    time.sleep(1.0)

    if not hdu.burn_arduino_firmware(port, sketch_name, a.repo_path, config_file=acli_path):
        print(Fore.RED + f"Failed to burn Arduino Sketch:{sketch_name} to port:{port}." + Style.RESET_ALL)
        sys.exit(1)
    print(Fore.GREEN + f"Burned Arduino Sketch:{sketch_name} Successfully to port:{port}." + Style.RESET_ALL)
