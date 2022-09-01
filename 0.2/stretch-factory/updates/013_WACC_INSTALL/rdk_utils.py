#!/usr/bin/env python
import os
from subprocess import Popen, PIPE
import fcntl
import subprocess
import sys

def add_arduino_pcba(device_name,is_stepper=False):
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

def add_dynamixel_pcba(device_name):
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



def exec_process(cmdline, silent, input=None, **kwargs):
    """Execute a subprocess and returns the returncode, stdout buffer and stderr buffer.
       Optionally prints stdout and stderr while running."""
    try:
        sub = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, **kwargs)
        stdout, stderr = sub.communicate(input=input)
        returncode = sub.returncode
        if not silent:
            sys.stdout.write(stdout)
            sys.stderr.write(stderr)
    except OSError,e:
        if e.errno == 2:
            raise RuntimeError('"%s" is not present on this system' % cmdline[0])
        else:
            raise
    if returncode != 0:
        raise RuntimeError('Got return value %d while executing "%s", stderr output was:\n%s' % (returncode, " ".join(cmdline), stderr.rstrip("\n")))
    return stdout

def reset_arduino_usb():
    USBDEVFS_RESET = 21780
    lsusb_out = Popen("lsusb | grep -i %s"%'Arduino', shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()
    while len(lsusb_out):
        bus = lsusb_out[1]
        device = lsusb_out[3][:-1]
        try:
            print('Resetting Arduino. Bus:',bus, 'Device: ',device)
            f = open("/dev/bus/usb/%s/%s" % (bus, device), 'w', os.O_WRONLY)
            fcntl.ioctl(f, USBDEVFS_RESET, 0)
        except Exception as msg:
            print("failed to reset device: %s"%msg)
        lsusb_out=lsusb_out[8:]

def reset_FTDI_usb():
    USBDEVFS_RESET = 21780
    lsusb_out = Popen("lsusb | grep -i %s"%'Future', shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()
    while len(lsusb_out):
        bus = lsusb_out[1]
        device = lsusb_out[3][:-1]
        try:
            print('Resetting FTDI. Bus:',bus, 'Device: ',device)
            f = open("/dev/bus/usb/%s/%s" % (bus, device), 'w', os.O_WRONLY)
            fcntl.ioctl(f, USBDEVFS_RESET, 0)

        except Exception as msg:
            print("failed to reset device: %s"%msg)
        lsusb_out=lsusb_out[15:]

def modify_bashrc(env_var,env_var_val):
    f=open(os.environ['HOME']+'/.bashrc','r')
    x=f.readlines()
    x_out=''
    for xx in x:
        if xx.find(env_var)>0:
            x_out=x_out+'export '+env_var+'='+env_var_val+'\n'
        else:
            x_out=x_out+xx
    f.close()
    f=open(os.environ['HOME']+'/.bashrc','w')
    f.write(x_out)
    f.close()


def add_arduino_udev_line(device_name, serial_no):
    from stretch_body.hello_utils import get_fleet_directory
    f = open(get_fleet_directory()+'udev/95-hello-arduino.rules', 'r')
    x = f.readlines()
    x_out = ''
    overwrite=False
    uline = 'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="804d",MODE:="0666", ATTRS{serial}=="'+serial_no + '", SYMLINK+="'+device_name+'", ENV{ID_MM_DEVICE_IGNORE}="1"\n'
    for xx in x:
        if xx.find(device_name) > 0 and xx[0]!='#':
            overwrite=True
            x_out = x_out + uline
            print 'Overwriting existing entry...'
        else:
            x_out = x_out + xx
    if not overwrite:
        print 'Creating new entry...'
        x_out = x_out + uline
    #print uline
    f.close()
    f = open(get_fleet_directory()+'udev/95-hello-arduino.rules', 'w')
    f.write(x_out)
    f.close()

def add_ftdi_udev_line(device_name, serial_no):
    from stretch_body.hello_utils import get_fleet_directory
    f = open(get_fleet_directory()+'udev/99-hello-dynamixel.rules', 'r')
    x = f.readlines()
    x_out = ''
    overwrite=False
    uline = 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTR{device/latency_timer}="1", ATTRS{serial}=="'+serial_no+'", SYMLINK+="'+device_name+'"'
    for xx in x:
        if xx.find(device_name) > 0 and xx[0]!='#':
            overwrite=True
            x_out = x_out + uline +'\n'
            print 'Overwriting existing entry...'
        else:
            x_out = x_out + xx
    if not overwrite:
        print 'Creating new entry...'
        x_out = x_out + uline + '\n'
    #print uline
    f.close()
    f = open(get_fleet_directory()+'udev/99-hello-dynamixel.rules', 'w')
    f.write(x_out)
    f.close()