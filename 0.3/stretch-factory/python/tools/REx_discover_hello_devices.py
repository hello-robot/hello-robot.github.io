#!/usr/bin/env python3
import os
import stretch_body.stepper
from stretch_body.dynamixel_XL430 import DynamixelXL430, DynamixelCommError
import stretch_body.hello_utils as hu
from stretch_body.device import Device
import click
import pprint
from stretch_factory import hello_device_utils as hdu
from stretch_body.robot_params import RobotParams
import argparse
import sys

hu.print_stretch_re_use()

parser = argparse.ArgumentParser(
    description="Find and map all the robot-specific USB devices (i.e. stepper motors: {Lift, Arm, Left wheel, "
                "Right wheel} and dynamixel servos: {Head,Wrist/End-of-arm}). Then assign the found devices to the "
                "robot by updating the UDEV rules and stretch configuration files.This tool can be utilized everytime "
                "a PCBA, motor assembly is replaced or if any /dev/hello-* devices go missing from the  USB bus.")

group = parser.add_mutually_exclusive_group()
group.add_argument("--discover",
                   help="Discover the devices by set of user instructions and re-assign them to the robot",
                   action="store_true")
group.add_argument("--list_tty",
                   help="Display the currently available devices's info(model,path,serial,vendor) present in /dev/ttyACM* and /dev/ttyUSB*",
                   action="store_true")
parser.add_argument("--arduino", help="Discover arduino/stepper devices only.", action="store_true")
parser.add_argument("--dynamixel", help="Discover Dynamixel servo motor devices only.", action="store_true")
args = parser.parse_args()


class DiscoverHelloDevices:
    def __init__(self):
        self.all_tty_devices = hdu.find_tty_devices()
        self.hello_arduino_devices = {}
        self.hello_dxl_devices = {}

        self.hello_stepper_devices = {}
        self.hello_stepper_sns = {"hello-motor-lift": None,
                                  "hello-motor-arm": None,
                                  "hello-motor-right-wheel": None,
                                  "hello-motor-left-wheel": None}
        self.hello_dxl_sns = {"hello-dynamixel-head": None,
                              "hello-dynamixel-wrist": None}
        self.hello_usb_alias = {}
        self.hello_pimu_sn = {'hello-pimu': None}
        self.hello_wacc_sn = {'hello-wacc': None}
        self.robot_tool = RobotParams.get_params()[1]['robot']['tool']

    def get_all_stepper_poses(self):
        poses = {}
        for k in list(self.hello_stepper_devices.keys()):
            try:
                motor = stretch_body.stepper.Stepper(usb=k, name='hello-motor-left-wheel')
                self.assertTrue(motor.startup(), "Unable to start stepper motor at {}".format(k))
                motor.pull_status()
                poses[k] = motor.status['pos']
                motor.stop()
            except Exception as e:
                print(click.style(f"Unable to get pose from stepper at port:{k}.\nError: {e}",fg="red"))
                poses[k] = None
        return poses

    def get_base_wheels_poses(self, left_wheel, right_wheel):
        poses = {'left_wheel': None, 'right_wheel': None}
        left_motor = stretch_body.stepper.Stepper(usb=left_wheel, name='hello-motor-left-wheel')
        right_motor = stretch_body.stepper.Stepper(usb=right_wheel, name='hello-motor-left-wheel')
        self.assertTrue(left_motor.startup())
        self.assertTrue(right_motor.startup())
        left_motor.pull_status()
        right_motor.pull_status()
        poses['left_wheel'] = left_motor.status['pos']
        poses['right_wheel'] = right_motor.status['pos']
        left_motor.stop()
        right_motor.stop()
        return poses

    def get_moved_motor(self, start_poses, end_poses):
        pos_diff_thresh = 0.8
        moved_motors = {}
        for k in list(start_poses.keys()):
            if start_poses[k] is not None and end_poses[k] is not None:
                pos_diff = start_poses[k] - end_poses[k]
                if abs(pos_diff) > pos_diff_thresh:
                    moved_motors[k] = True
                    print("Motor in {} was moved by {}".format(k, pos_diff))
            else:
                moved_motors[k] = False
        return moved_motors

    def assertEqual(self, a, b, msg=None):
        if a == b:
            return True
        else:
            if msg is not None:
                print(click.style(msg, fg='red'))
            return False

    def assertIsNotNone(self, a, msg=None):
        if a is None:
            if msg is not None:
                print(click.style(msg, fg='red'))
            return False
        else:
            return True

    def assertTrue(self, a, msg=None):
        if a == True:
            return True
        else:
            if msg is not None:
                print(click.style(msg, fg='red'))
            return False

    def get_arduino_devices(self):
        """
        Populate all the devices with Arduino_LLC vendor and grab unique device model serial numbers
        """
        for k in self.all_tty_devices.keys():
            if self.all_tty_devices[k]['vendor'] == 'Arduino_LLC':
                dev = self.all_tty_devices[k]

                self.hello_arduino_devices[k] = dev

                if dev['model'] == 'Hello_Stepper':
                    self.hello_stepper_devices[k] = dev
                elif dev['model'] == 'Hello_Pimu':
                    self.hello_pimu_sn['hello-pimu'] = dev['serial']
                elif dev['model'] == 'Hello_Wacc':
                    self.hello_wacc_sn['hello-wacc'] = dev['serial']

        print("Found {} Arduino devices:".format(len(self.hello_arduino_devices.keys())))
        pprint.pprint(self.hello_arduino_devices)
        print("\n")
        print("Found {} hello_stepper devices.".format(len(self.hello_stepper_devices.keys())))
        self.assertEqual(len(self.hello_arduino_devices.keys()), 6, "Unable to find the correct no. arduino devices.")
        self.assertEqual(len(self.hello_stepper_devices.keys()), 4,
                         "Unable to find the correct no. hello_stepper devices.")

        if self.assertIsNotNone(self.hello_pimu_sn['hello-pimu'], "Unable to find PIMU Serial"):
            print(click.style("Found hello-pimu serial: {}".format(self.hello_pimu_sn['hello-pimu']), fg='green'))

        if self.assertIsNotNone(self.hello_wacc_sn['hello-wacc'], "Unable to find WACC Serial"):
            print(click.style("Found hello-wacc serial: {}".format(self.hello_wacc_sn['hello-wacc']), fg='green'))

    def get_dxl_devices(self):
        """
        Populate all the devices with FTDI vendor (dynamixel driver)
        """
        for k in self.all_tty_devices.keys():
            if self.all_tty_devices[k]['vendor'] == 'FTDI' or self.all_tty_devices[k]['vendor_id'] == '0403' or  self.all_tty_devices[k]['model_id'] == '6001':
                self.hello_dxl_devices[k] = self.all_tty_devices[k]
        print("Found {} dxl devices:".format(len(self.hello_dxl_devices.keys())))
        pprint.pprint(self.hello_dxl_devices, width=40)

        self.assertEqual(len(self.hello_dxl_devices.keys()), 2, "Unable to find the correct no. FTDI devices.")

    def get_lift_sn(self):
        """
        Find the Serial number of the Lift motor serial by detecting a manual movement
        """
        input(click.style("WARNING: The Lift would drop, make sure the clamp is underneath lift. Then hit ENTER",
                          fg="yellow", bold=True))
        start_pose = self.get_all_stepper_poses()
        input(click.style("Move the Lift joint manually to another position and place the clamp underneath when done. Then hit ENTER",
                          fg="blue",
                          bold=True))
        end_pose = self.get_all_stepper_poses()
        moved_motors = self.get_moved_motor(start_pose, end_pose)
        cnt = list(moved_motors.values()).count(True)
        self.assertEqual(cnt, 1, "More than one motor or none moved")
        lift_sn = None
        for k in list(moved_motors.keys()):
            if moved_motors[k]:
                lift_sn = self.all_tty_devices[k]['serial']
                self.hello_usb_alias[k] = "/dev/hello-motor-lift"
        if self.assertIsNotNone(lift_sn, "Unable to find Lift Serial"):
            print(click.style("Found hello-motor-lift serial: {}".format(lift_sn), fg='green'))
        self.hello_stepper_sns['hello-motor-lift'] = lift_sn

    def get_arm_sn(self):
        """
        Find the Serial number of the Arm motor serial by detecting a manual movement
        """
        start_pose = self.get_all_stepper_poses()
        input(click.style("Move the Arm joint manually to another position and hit ENTER", fg="blue", bold=True))
        end_pose = self.get_all_stepper_poses()
        moved_motors = self.get_moved_motor(start_pose, end_pose)
        cnt = list(moved_motors.values()).count(True)
        self.assertEqual(cnt, 1, "More than one motor or none moved")
        arm_sn = None
        for k in list(moved_motors.keys()):
            if moved_motors[k]:
                arm_sn = self.all_tty_devices[k]['serial']
                self.hello_usb_alias[k] = "/dev/hello-motor-arm"
        self.hello_stepper_sns['hello-motor-arm'] = arm_sn
        if self.assertIsNotNone(arm_sn, "Unable to find Arm Serial"):
            print(click.style("Found hello-motor-arm serial: {}".format(arm_sn), fg='green'))

    def get_wheels_sn(self):
        """
        Find the Serial number of the wheel motors serial by detecting a manual movement
        """
        start_pose = self.get_all_stepper_poses()
        input(click.style("Move the Base backward manually to another position and hit ENTER", fg="blue", bold=True))
        end_pose = self.get_all_stepper_poses()
        moved_motors = self.get_moved_motor(start_pose, end_pose)
        cnt = list(moved_motors.values()).count(True)
        if self.assertEqual(cnt, 2, f"No. motors moved={cnt}. Wrong number of motors or none moved"):
            wheel_motors = [k for k in list(moved_motors.keys()) if moved_motors[k]]

            """Both the unknown sides wheel motor poses are pulled in 'hello-motor-left' wheel configuration. When the 
            robot is manually back driven forward, the wheel pose difference is -ve and +ve for the left and right wheel 
            respectively. This logic is used to differentiate left wheel from right wheel"""

            start_pos1 = self.get_base_wheels_poses(wheel_motors[0], wheel_motors[1])
            input(click.style("Move the base forward manually to another position and hit ENTER", fg="blue", bold=True))
            end_pose1 = self.get_base_wheels_poses(wheel_motors[0], wheel_motors[1])
            assumed_left_diff = start_pos1['left_wheel'] - end_pose1['left_wheel']
            assumed_right_diff = start_pos1['right_wheel'] - end_pose1['right_wheel']
            left_sn = None
            right_sn = None
            if assumed_left_diff < 0 < assumed_right_diff:
                left_sn = self.all_tty_devices[wheel_motors[0]]['serial']
                right_sn = self.all_tty_devices[wheel_motors[1]]['serial']
                self.hello_usb_alias[wheel_motors[0]] = "/dev/hello-motor-left-wheel"
                self.hello_usb_alias[wheel_motors[1]] = "/dev/hello-motor-right-wheel"
            else:
                left_sn = self.all_tty_devices[wheel_motors[1]]['serial']
                right_sn = self.all_tty_devices[wheel_motors[0]]['serial']
                self.hello_usb_alias[wheel_motors[1]] = "/dev/hello-motor-left-wheel"
                self.hello_usb_alias[wheel_motors[0]] = "/dev/hello-motor-right-wheel"

            self.hello_stepper_sns['hello-motor-left-wheel'] = left_sn
            self.hello_stepper_sns['hello-motor-right-wheel'] = right_sn
            if self.assertIsNotNone(left_sn, "Unable to find left wheel serial."):
                print(click.style("Found hello-motor-left-wheel serial: {}".format(left_sn), fg='green'))
            if self.assertIsNotNone(right_sn, "Unable to find right wheel serial."):
                print(click.style("Found hello-motor-right-wheel serial: {}".format(right_sn), fg='green'))

    def get_servo_ids(self, port, baud_to=115200):
        found_ids = []
        print('\nScanning for servo at port: {}'.format(port))
        print('----------------------------------------------------------')
        b = baud_to
        for id in range(20):
            print("Checking at ID %d and baud %d" % (id, b))
            m = DynamixelXL430(id, port, baud=b)
            m.logger.disabled = True
            try:
                if m.startup():
                    print('Found servo at ID %d and Baud %d' % (id, b))
                    found_ids.append(id)
            except DynamixelCommError:
                print("ping failed for ID: " + str(id))
                continue
        return found_ids

    def get_dynamixel_sns(self):
        """
        Scan the dxl ports for Servo IDs and find the appropriate dynamixel serial
        """
        found_ids = {}
        for k in list(self.hello_dxl_devices.keys()):
            found_ids[k] = self.get_servo_ids(k)
            print("FOUND Servo IDs: {} at port: {}".format(found_ids[k], k))

        head_sn = None
        wrist_sn = None
        for k in list(found_ids.keys()):
            if found_ids[k] == [11, 12]:
                head_sn = self.all_tty_devices[k]['serial']
                self.hello_usb_alias[k] = "/dev/hello-dynamixel-head"
            if 13 in found_ids[k]:
                wrist_sn = self.all_tty_devices[k]['serial']
                self.hello_usb_alias[k] = "/dev/hello-dynamixel-wrist"
        self.hello_dxl_sns["hello-dynamixel-head"] = head_sn
        self.hello_dxl_sns["hello-dynamixel-wrist"] = wrist_sn
        print("\n")
        if self.assertIsNotNone(head_sn, "Unable to find Head Serial"):
            print(click.style("Found hello-dynamixel-head serial: {}".format(head_sn), fg='green'))
        if self.assertIsNotNone(wrist_sn, "Unable to find Wrist Serial"):
            print(click.style("Found hello-dynamixel-wrist serial: {}".format(wrist_sn), fg='green'))

    def push_sns_to_udev_rules(self):
        """
        Update the Udev files with the found Serial numbers for hello* devices
        """
        self.push_arduino_sns_to_udev_rules()
        self.push_dynamixel_sns_to_udev_rules()
        # print(click.style('UDEV rules and stretch configuration files updated', fg='green', bold=True))

    def push_dynamixel_sns_to_udev_rules(self):
        os.system("chmod -R 777 ~/stretch_user")
        print("Assigning FTDI devices SN to robot....")
        for k in list(self.hello_dxl_sns.keys()):
            try:
                if self.hello_dxl_sns[k]:
                    hdu.add_ftdi_udev_line(device_name=k, serial_no=self.hello_dxl_sns[k],
                                        fleet_dir=hu.get_fleet_directory())
                else:
                    print(click.style('Unable to find SN of {}. Skipping.'.format(k), fg='red'))
            except Exception as err:
                print(click.style('ERROR [{}]: {}'.format(k, str(err)), fg='red'))
        print(click.style('UDEV rules and stretch configuration files specific to Dynamixels updated', fg='green',
                          bold=True))

    def push_arduino_sns_to_udev_rules(self):
        os.system("chmod -R 777 ~/stretch_user")
        print("Assigning Stepper motor SN to robot....")

        for k in list(self.hello_stepper_sns.keys()):
            try:
                if self.hello_stepper_sns[k]:
                    hdu.add_arduino_udev_line(device_name=k, serial_no=self.hello_stepper_sns[k],
                                            fleet_dir=hu.get_fleet_directory())
                    dev = Device(k)
                    dev.write_configuration_param_to_YAML("{}.serial_no".format(k), self.hello_stepper_sns[k],
                                                        hu.get_fleet_directory())
                else:
                    print(click.style('Unable to find SN of {}. Skipping.'.format(k), fg='red'))
            except Exception as err:
                print(click.style('ERROR [{}]: {}'.format(k, str(err)), fg='red'))

        print("Assigning Pimu and Wacc SN to robot....")
        try:
            if self.hello_pimu_sn['hello-pimu']:
                hdu.add_arduino_udev_line(device_name="hello-pimu", serial_no=self.hello_pimu_sn['hello-pimu'],
                                        fleet_dir=hu.get_fleet_directory())
            else:
                print(click.style('Unable to find SN of {}. Skipping.'.format('hello-pimu'), fg='red'))
        except Exception as err:
            print(click.style('ERROR [{}]: {}'.format("hello-pimu", str(err)), fg='red'))

        try:
            if self.hello_wacc_sn['hello-wacc']:
                hdu.add_arduino_udev_line(device_name="hello-wacc", serial_no=self.hello_wacc_sn['hello-wacc'],
                                        fleet_dir=hu.get_fleet_directory())
            else:
                print(click.style('Unable to find SN of {}. Skipping.'.format('hello-wacc'), fg='red'))
        except Exception as err:
            print(click.style('ERROR [{}]: {}'.format("hello-wacc", str(err)), fg='red'))
        print(click.style('UDEV rules and stretch configuration files specific to arduino devices updated', fg='green',
                          bold=True))
    
    def push_udev_rules_to_etc(self,arduino=True,dynamixel=True):
        print("\nPushing Udev files to /etc/udev/rules.d/")
        if arduino:
            os.system("sudo cp {}udev/95-hello-arduino.rules /etc/udev/rules.d/".format(hu.get_fleet_directory()))
            os.system("sudo cp {}udev/95-hello-arduino.rules /etc/hello-robot/{}/udev/".format(hu.get_fleet_directory(),hu.get_fleet_id()))
        if dynamixel:
            os.system("sudo cp {}udev/99-hello-dynamixel.rules /etc/udev/rules.d/".format(hu.get_fleet_directory()))
            os.system("sudo cp {}udev/99-hello-dynamixel.rules /etc/hello-robot/{}/udev/".format(hu.get_fleet_directory(),hu.get_fleet_id()))
        os.system("sudo udevadm control --reload; sudo udevadm trigger")


    def run(self):
        if len(list(self.all_tty_devices.keys())) == 0:
            print(click.style("No ttyACM* or ttyUSB* devices were found in the USB bus.", fg="red"))
            sys.exit()
        self.get_arduino_devices()
        self.get_dxl_devices()
        self.get_lift_sn()
        self.get_arm_sn()
        self.get_wheels_sn()
        self.get_dynamixel_sns()
        self.push_sns_to_udev_rules()
        self.push_udev_rules_to_etc()

    def run_dynamixel(self):
        if len(list(self.all_tty_devices.keys())) == 0:
            print(click.style("No ttyACM* or ttyUSB* devices were found in the USB bus.", fg="red"))
            sys.exit()
        self.get_dxl_devices()
        self.get_dynamixel_sns()
        self.push_dynamixel_sns_to_udev_rules()
        self.push_udev_rules_to_etc(arduino=False)

    def run_arduino(self):
        if len(list(self.all_tty_devices.keys())) == 0:
            print(click.style("No ttyACM* or ttyUSB* devices were found in the USB bus.", fg="red"))
            sys.exit()
        self.get_arduino_devices()
        self.get_lift_sn()
        self.get_arm_sn()
        self.get_wheels_sn()
        self.push_arduino_sns_to_udev_rules()
        self.push_udev_rules_to_etc(dynamixel=False)


if args.list_tty:
    discover_hello_devices = DiscoverHelloDevices()
    pprint.pprint(discover_hello_devices.all_tty_devices)
elif args.discover and args.arduino:
    discover_hello_devices = DiscoverHelloDevices()
    discover_hello_devices.run_arduino()
elif args.discover and args.dynamixel:
    discover_hello_devices = DiscoverHelloDevices()
    discover_hello_devices.run_dynamixel()
elif args.discover:
    discover_hello_devices = DiscoverHelloDevices()
    discover_hello_devices.run()
else:
    parser.print_help()
