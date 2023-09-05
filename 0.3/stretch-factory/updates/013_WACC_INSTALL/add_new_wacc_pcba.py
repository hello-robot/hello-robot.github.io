#!/usr/bin/env python
#Set relative path to utils
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'utils'))
import rdk_utils   # Importing my_module
import time
import stretch_body.device

import subprocess

def add_arduino(device_name,is_stepper=False):
    dev_found=False
    rdk_utils.exec_process(['sudo','dmesg','-c'],True)
    print 'Plug / Reset in Arduino device now...'
    print 'Press return when done'
    raw_input()
    time.sleep(1.0)
    dmesg_data = rdk_utils.exec_process(['sudo','dmesg','-c'],True)
    pidx=0
    for line in dmesg_data.split('\n'):
        pidx=max(0,pidx-1)
        ff=line.find('Arduino LLC')
        if ff>0:
            pidx=2
        if line.find('SerialNumber') and pidx==1:
            sn=line[line.find('SerialNumber')+14:]
            print '---------------------------'
            print 'Found Arduino device with SerialNumber',sn
            print 'Writing UDEV for ',device_name,sn
            rdk_utils.add_arduino_udev_line(device_name,sn)
            if is_stepper:
                print 'Setting serial number in YAML for',device_name
                d = stretch_body.device.Device(req_params=False)
                d.robot_params[device_name]['serial_no'] = sn
                d.write_device_params(device_name, d.robot_params[device_name])
            dev_found=True
    if not dev_found:
        print 'No Arduino device device found'

def add_dynamixel(device_name):
    dev_found = False
    rdk_utils.exec_process(['sudo','dmesg','-c'],True)
    print 'Plug / Reset Dynamixel device now...'
    print 'Press return when done'
    raw_input()
    time.sleep(1.0)
    dmesg_data = rdk_utils.exec_process(['sudo','dmesg','-c'],True)
    print dmesg_data
    pidx=0
    for line in dmesg_data.split('\n'):
        pidx=max(0,pidx-1)
        ff=line.find('Product: FT232R USB')
        if ff>0:
            pidx=3
        if line.find('SerialNumber') and pidx==1:
            sn=line[line.find('SerialNumber')+14:]
            print '---------------------------'
            print 'Found Dynamixel device with SerialNumber',sn
            print 'Writing UDEV for ',sn
            dev_found = True
            rdk_utils.add_ftdi_udev_line(device_name,sn)
    if not dev_found:
        print 'No Dynamixel device found'



# #####################################################
print '----------------------'
print 'Adding WACC PCBA to robot: ',os.environ['HELLO_FLEET_ID']

add_dynamixel('hello-dynamixel-wrist')
add_arduino('hello-wacc')

#Now copy over udev and restart USB
subprocess.call('sudo cp $HELLO_FLEET_PATH/$HELLO_FLEET_ID/udev/*.rules /etc/udev/rules.d',shell=True)
subprocess.call('sudo cp $HELLO_FLEET_PATH/$HELLO_FLEET_ID/udev/*.rules /etc/hello-robot/$HELLO_FLEET_ID/udev',shell=True)
subprocess.call('sudo udevadm control --reload',shell=True)



