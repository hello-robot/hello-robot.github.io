import serial.tools.list_ports
from subprocess import Popen, PIPE
import usb.core
import os
import time

class StretchDeviceMgmt:
    """
    This class will collect the information for the Hello Robot connected USB devices into a single dictionary.
    Use of usb.core requires that the calling process as read permission to the device. By default this is set to root.
    Therefore, you may need to add the fallowing to a udev rule if it is not already there (eg in a file
    /etc/udev/rules.d/94-hello-usb.rules :

    SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", MODE="0664", GROUP="plugdev"

    Also see: https://stackoverflow.com/questions/31992058/how-can-i-comunicate-with-this-device-using-pyusb/31994168#31994168
    """
    def __init__(self,device_names=None):
        self.comports= serial.tools.list_ports.comports()
        if device_names==None:
            self.device_names = ['hello-motor-arm',
                                 'hello-motor-lift',
                                 'hello-motor-right-wheel',
                                 'hello-motor-left-wheel',
                                 'hello-dynamixel-wrist',
                                 'hello-dynamixel-head',
                                 'hello-pimu',
                                 'hello-wacc']
        else:
            self.device_names=device_names
        self.device_info={}
        for d in self.device_names:
            self.device_info[d]={'device':None,'info':None, 'core':None}

        self.valid=True

        #Build mapping between symlink and device name
        n_match=0
        lsdev=Popen("ls -ltr /dev/hello*", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().split(b'\n')
        for name in self.device_info.keys():
            for line in lsdev:
                if line.find(name.encode())>=0:
                    map=line[line.find(name.encode()):] #eg: hello-motor-arm -> ttyACM4
                    device=map[map.find(b'->')+3:] #eg ttyACM
                    self.device_info[name]['device']=device
                    n_match=n_match+1
        if not n_match==len(self.device_info.keys()):
            print('ls parse: Failed to match all devices for StretchSerialInfo')
            print(self.device_info)
        for c in self.comports:
            for name in self.device_info.keys():
                if c.device[5:].encode()==self.device_info[name]['device']:
                    self.device_info[name]['info']=c
        devs = []
        if self.check_udev_rules():
            all = usb.core.find(find_all=True)
            for dev in all:
                if dev.idVendor == 0x2341 and dev.idProduct == 0x804d:
                    devs.append(dev)
                if dev.idVendor == 0x0403 and dev.idProduct == 0x6001:
                    devs.append(dev)
            n_match=0
            for name in self.device_info.keys():
                for d in devs:
                    if d is not None and self.device_info[name]['info'] is not None:
                        try:
                            if self.device_info[name]['info'].serial_number == d.serial_number:
                                n_match=n_match+1
                                self.device_info[name]['core']=d
                        except ValueError:
                            print('ValueError. %s on serial number read. Reboot machine and try again.'%name)
                            self.valid=False
                            break
                if not self.valid:
                    break
        else:
            print('Failed to set udev rules for StretchDeviceMgmt')
            self.valid=False
        if not n_match==len(self.device_info.keys()):
            print('usb.core parse: Failed to match all devices for StretchSerialInfo')
            self.valid=False

    def check_udev_rules(self):
        if  os.path.exists('/etc/udev/rules.d/94-hello-usb.rules'):
            return True
        print('File 94-hello-usb.rules missing from UDEV. Installing now.')
        f= open('/tmp/94-hello-usb.rules',"w+")
        f.write("SUBSYSTEM==\"usb\", ENV{DEVTYPE}==\"usb_device\", MODE=\"0664\", GROUP=\"plugdev\"")
        f.close()
        if os.system('sudo mv /tmp/94-hello-usb.rules /etc/udev/rules.d')!=0:
            return False
        if os.system('sudo udevadm control --reload-rules')!=0:
            return False
        if os.system('sudo udevadm trigger')!=0:
            return False
        time.sleep(1.0) #Give time to reload
        return True


    def pretty_print(self):
        print('---- Stretch Serial Info ----')
        for name in self.device_info.keys():
            print('-----------------------------------------')
            print('%s : %s'%(name,self.device_info[name]['device']))
            if self.device_info[name]['info'] is not None:
                print('Serial: %s'%self.device_info[name]['info'].serial_number)
                print('Description: %s' % self.device_info[name]['info'].description)
                print('Location: %s' % self.device_info[name]['info'].location)

    def reset_all(self,verbose=True):
        if not self.valid:
            return False
        success=True
        for name in self.device_info.keys():
            success=success and self.reset(name,verbose)
        return success
    def reset(self,name,verbose=True):
        if not self.valid:
            return False
        if self.device_info[name]['core'] is not None:
            if verbose:
                print('Resetting %s' % name)
            self.device_info[name]['core'].reset()
            return True
        else:
            print('Not able to reset device %s'%name)
            return False

