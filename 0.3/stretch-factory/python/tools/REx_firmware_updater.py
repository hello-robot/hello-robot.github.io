#!/usr/bin/env python
import argparse
from stretch_factory.firmware_available import FirmwareAvailable
from stretch_factory.firmware_recommended import FirmwareRecommended
from stretch_factory.firmware_installed import FirmwareInstalled
from stretch_factory.firmware_updater import FirmwareUpdater
import os
import click
import stretch_factory.hello_device_utils as hdu

parser = argparse.ArgumentParser(description='Upload Stretch firmware to microcontrollers')

group = parser.add_mutually_exclusive_group()
group.add_argument("--current", help="Display the currently installed firmware versions", action="store_true")
group.add_argument("--available", help="Display the available firmware versions", action="store_true")
group.add_argument("--recommended", help="Display the recommended firmware", action="store_true")
group.add_argument("--install", help="Install the recommended firmware", action="store_true")
group.add_argument("--install_version", help="Install a specific firmware version", action="store_true")
group.add_argument("--install_branch", help="Install the HEAD of a specific branch", action="store_true")
group.add_argument("--install_path", help="Install the firmware on the provided path (eg ./stretch_firmware/arduino)", type=str)
group.add_argument("--resume", help="Resume an install in progress", action="store_true")
group.add_argument("--mgmt", help="Display overview on firmware management", action="store_true")
parser.add_argument("--map", help="Print mapping from ttyACMx to Hello device", action="store_true")

parser.add_argument("--pimu", help="Upload Pimu firmware", action="store_true")
parser.add_argument("--wacc", help="Upload Wacc firmware", action="store_true")
parser.add_argument("--arm", help="Upload Arm Stepper firmware", action="store_true")
parser.add_argument("--lift", help="Upload Lift Stepper firmware", action="store_true")
parser.add_argument("--left_wheel", help="Upload Left Wheel Stepper firmware", action="store_true")
parser.add_argument("--right_wheel", help="Upload Right Wheel Stepper firmware", action="store_true")
parser.add_argument("--no_prompts", help="Proceed without prompts", action="store_true")
parser.add_argument("--verbose", help="Verbose output", action="store_true")
args = parser.parse_args()

mgmt = """
FIRMWARE MANAGEMENT
--------------------
The Stretch Firmware is managed by Git tags. 

The repo is tagged with versions as <Board>.v<Major>.<Minor>.<Bugfix><Protocol>
For example Pimu.v0.0.1p0

This same version is included the Arduino file Common.h and is burned to the board EEPROM. It 
can be read from Stretch Body as <device>.board_info

Each Stretch Body device (Stepper, Wacc, Pimu) includes a dictionary: supported_protocols
For example, stepper.supported_protocols.keys()=['p0','p1']

The updater will determine the available firmware versions given the current Stretch Body that is installed on 
the default Python path.

The updater will then query each device to determine what firmware is currently flashed to the boards. It can then
recommend updates to the user.

WHEN UPDATING FIRMWARE CODE
----------------------
After updating the firmware
* Increment the version / protocol in the device's Common.h', eg
  #define FIRMWARE_VERSION "Pimu.v0.0.5p0"
* When updating the protocol version increment the <Minor> version as well, for example
    Pimu.v0.0.5p0 --> Pimu.v0.1.0p1
* Tag with the full version name that matches Common.h , eg
  git tag -a Pimu.v0.0.5p1 -m "Pimu bugfix of foo"
*Push tag to remote
  git push origin --tags
* Check the code in to stretch_firmware

If there was a change in protocol number, also update Stretch Body
accordingly. For example in stepper.supported_protocols, add {'p1': Stepper_Protocol_P1}

TAGGING
--------
https://git-scm.com/book/en/v2/Git-Basics-Tagging

To see available tags
  git log --pretty=oneline 

To tag an older commit
  git tag -a Pimu.v0.0.5p1 <hash> -m "Pimu bugfix of foo"

Push tags
  git push origin --tags

Delete tags
  git tag -d Pimu.v0.0.5p1
  git push origin --delete  Pimu.v0.0.5p1
USER EXPERIENCE
----------------
The user may update Stetch Body version from time to time. After installing
a new version of Stretch Body, this firmware updater tools should be run. 
"""

if args.arm or args.lift or args.wacc or args.pimu or args.left_wheel or args.right_wheel:
    use_device={'hello-motor-lift':args.lift,'hello-motor-arm':args.arm, 'hello-motor-right-wheel':args.right_wheel, 'hello-motor-left-wheel':args.left_wheel,'hello-pimu':args.pimu,'hello-wacc':args.wacc}
else:
    use_device = {'hello-motor-arm': True, 'hello-motor-right-wheel': True, 'hello-motor-left-wheel': True, 'hello-pimu': True, 'hello-wacc': True,'hello-motor-lift': True}

if args.mgmt:
    print(mgmt)
    exit()

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
        exit()

if args.current:
    c = FirmwareInstalled(use_device)
    c.pretty_print()
    exit()

if args.recommended:
    r = FirmwareRecommended(use_device)
    r.pretty_print()
    r.print_recommended_args()
    exit()

if args.available:
    a = FirmwareAvailable(use_device)
    a.pretty_print()
    exit()


if args.resume or args.install or args.install_version or args.install_branch or args.install_path:
    u = FirmwareUpdater(use_device, args)
    success = u.run()
    exit(0 if success else 1)
else:
    parser.print_help()

