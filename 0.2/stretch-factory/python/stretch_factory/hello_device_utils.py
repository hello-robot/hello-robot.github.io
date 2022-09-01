#!/usr/bin/env python3
from __future__ import print_function
import time
import requests
import os
from subprocess import Popen, PIPE
import fcntl
import subprocess
import sys

from subprocess import Popen, PIPE
import usb.core
import stretch_body.hello_utils as hello_utils
import stretch_body.robot_params
import stretch_body.device

# ###################################
class stream_tee(object):
    # Based on https://gist.github.com/327585 by Anand Kunal
    # http://www.tentech.ca/2011/05/stream-tee-in-python-saving-stdout-to-file-while-keeping-the-console-alive/
    def __init__(self, stream1, stream2):
        self.stream1 = stream1
        self.stream2 = stream2
        self.__missing_method_name = None  # Hack!

    def __getattribute__(self, name):
        return object.__getattribute__(self, name)

    def __getattr__(self, name):
        self.__missing_method_name = name  # Could also be a property
        return getattr(self, '__methodmissing__')

    def __methodmissing__(self, *args, **kwargs):
        # Emit method call to the log copy
        callable2 = getattr(self.stream2, self.__missing_method_name)
        callable2(*args, **kwargs)

        # Emit method call to stdout (stream 1)
        callable1 = getattr(self.stream1, self.__missing_method_name)
        return callable1(*args, **kwargs)
# ###################################
def check_internet():
    url = "http://github.com"
    timeout = 10
    for i in range(10):
        print('Attempting to reach internet. Try %d of 10'%(i+1))
        try:
            requests.get(url, timeout=timeout)
            print("Connected to the GitHub")
            return {'success':1}
        except (requests.ConnectionError, requests.Timeout) as exception:
            print("Failed to establish internet connection.")
            time.sleep(0.5)
    return {'success':0}
# ###################################
def check_arduino_cli_install():
    """
    Return true if the arduino-cli is available
    """
    res=Popen('arduino-cli version', shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()[:11]
    if not(res==b'arduino-cli'):
        print('WARNING:---------------------------------------------------------------------------------')
        print('WARNING: Tool arduino_cli not installed. See stretch_install_dev.sh (Stretch Install repo)')
        print('WARNING:---------------------------------------------------------------------------------')
        print('')
        return False
    return True
# ###################################
def exec_process(cmdline, silent, input=None, **kwargs):
    """Execute a subprocess and returns the returncode, stdout buffer and stderr buffer.
       Optionally prints stdout and stderr while running."""
    try:
        sub = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, **kwargs)
        stdout, stderr = sub.communicate(input=input)
        returncode = sub.returncode
        if not silent:
            sys.stdout.write(stdout.decode('utf-8'))
            sys.stderr.write(stderr.decode('utf-8'))
    except OSError as e:
        if e.errno == 2:
            raise RuntimeError('"%s" is not present on this system' % cmdline[0])
        else:
            raise
    if returncode != 0:
        raise RuntimeError('Got return value %d while executing "%s", stderr output was:\n%s' % (returncode, " ".join(cmdline), stderr.rstrip(b"\n")))
    return stdout

# ###################################
def is_device_present(device):
    try:
        exec_process(['ls',device],True)
        return True
    except RuntimeError as e:
        return False

def find_steppers_on_bus():
    s=[]
    device_names=['/dev/hello-motor-arm', '/dev/hello-motor-lift', '/dev/hello-motor-right-wheel', '/dev/hello-motor-left-wheel']
    for d in device_names:
        if is_device_present(d):
            s.append(d)
    return s
# ###################################
def find_arduinos():
    devs = []
    all = usb.core.find(find_all=True)
    for dev in all:
        if dev.idVendor == 0x2341 and dev.idProduct == 0x804d:
            devs.append(dev)
    return devs

def find_ftdis():
    devs = []
    all = usb.core.find(find_all=True)
    for dev in all:
        if dev.idVendor == 0x0403 and dev.idProduct == 0x6001:
            devs.append(dev)
    return devs

# ###################################
def get_dmesg():
    return exec_process(['sudo', 'dmesg', '-c'], True)

def find_arduino():
    """
    Assumes Arduino data is already on dmesg
    """
    dmesg_data = get_dmesg()
    lines = dmesg_data.split(b'\n')
    for idx in range(len(lines)):
        if lines[idx].find(b'Product: Hello')>0:
            board=lines[idx][lines[idx].find(b'Product: ')+9:]
            idx=min(idx+2,len(lines)-1)
            if lines[idx].find(b'SerialNumber') >0:
                sn=lines[idx][lines[idx].find(b'SerialNumber')+14:]
                if lines[idx].find(b'SerialNumber') >0:
                    sn=lines[idx][lines[idx].find(b'SerialNumber')+14:]
                    idx=min(idx+1,len(lines)-1)
                    if lines[idx].find(b'ttyACM') > 0:
                        port = b'/dev/'+lines[idx][lines[idx].find(b'ttyACM'):lines[idx].find(b'ttyACM')+7] #only ttyACM0-9
                        if (is_device_present(port)):
                            print('---------------------------')
                            print('Found board %s with SerialNumber %s on port %s'%(board,sn,port))
                            return {'board':board,'serial':sn,'port':port}
    print('No Arduino device device found')
    return {'board':None,'serial':None,'port':None}

def find_ftdi_sn():
    try:
        x=exec_process([b'sudo',b'lsusb', b'-d', b'0403:6001',b'-v'], True).split(b'\n')
        for ln in x:
            if ln.find(b'iSerial')>=0:
                sn=ln.split(b' ')[-1]
                return sn
    except RuntimeError:
        print('FTDI device not found')
        return None

def find_shoulder_hub():
    lsusb_out = Popen("lsusb | grep '1a40:0101'", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()
    #Note: User periphs using Terminus may ID false positive
    if len(lsusb_out):
        bus = lsusb_out[1]
        device = lsusb_out[3][:-1]
        port= "/dev/bus/usb/%s/%s" % (bus, device)
        print('Found shoulder hub IC on bus at %s'%port)
        return {'success':1}
    else:
        print('Terminus USB2 hub not found')
        return {'success':0}

def find_ftdi_port():
    lsusb_out = Popen("lsusb | grep -i %s"%'Future', shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()
    if len(lsusb_out):
        bus = lsusb_out[1]
        device = lsusb_out[3][:-1]
        return "/dev/bus/usb/%s/%s" % (bus, device)
    else:
        print('FTDI port not found')
        return None
# ###################################
def compile_arduino_firmware(sketch_name,repo_path):
    """
    :param sketch_name: eg 'hello_stepper'
    :return T if success:
    """
    compile_command = 'arduino-cli compile --fqbn hello-robot:samd:%s %s/arduino/%s' % (sketch_name, repo_path, sketch_name)
    print(compile_command)
    c = Popen(compile_command, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip()
    return c.find(b'Sketch uses')>-1

def burn_arduino_firmware(port, sketch_name,repo_path):
    print('-------- Flashing firmware %s | %s ------------' % (port, sketch_name))
    port_name = Popen("ls -l " + port, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()[-1]
    port_name=port_name.decode("utf-8")
    if port_name is not None:
        upload_command = 'arduino-cli upload -p %s --fqbn hello-robot:samd:%s %s/arduino/%s' % (port_name, sketch_name,repo_path, sketch_name)
        print('Running: %s'%upload_command)
        u = Popen(upload_command, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip()
        uu = u.split(b'\n')
        #Pretty print the result
        for l in uu:
            k=l.split(b'\r')
            if len(k)==1:
                print(k[0].decode('utf-8'))
            else:
                for m in k:
                    print(m.decode('utf-8'))
        print('################')
        success=uu[-1]==b'CPU reset.'
        if not success:
            print('Firmware flash. Failed to upload to %s'% ( port))
        else:
            print('Success in firmware flash')
        return success
    else:
        print('Firmware flash. Failed to find device %s' % ( port))
        return False

def burn_bootloader(sketch):
    print('-------- Burning bootlader ------------')
    cmd = 'arduino-cli burn-bootloader -b hello-robot:samd:%s -P sam_ice' % sketch
    cmdl = ['arduino-cli', 'burn-bootloader', '-b', 'hello-robot:samd:%s' % sketch, '-P', 'sam_ice']
    print(cmd)
    u = subprocess.call(cmdl)
    return u==0

# ##################################

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

# ##############################################################

def run_firmware_flash(port,sketch,repo_path=''):
    #Return board serial # / success
    print('### Running Firmware Flash on port %s and sketch %s'%(port,sketch))
    print('###################################################################')
    if burn_bootloader(sketch):
        print('SUCCESS: Burned bootloader')
    else:
        print('FAIL: Could not burn bootlaoder')
        return  {'success':0,'sn':None}
    time.sleep(2.0)
    if is_device_present(port):
        print('SUCCESS: Found PCBA on %s'%port)
    else:
        print('FAIL: Did not find PCBA on %s'%port)
        return  {'success':0,'sn':None}

    if compile_arduino_firmware(sketch,repo_path):
        print('SUCCESS: Found compiled sketch %s' % sketch)
    else:
        print('FAIL: Could not compile sketch %s' % sketch)
        return  {'success':0,'sn':None}

    get_dmesg()
    if burn_arduino_firmware(port,sketch,repo_path):
        print('SUCCESS: Flashed sketch %s' % sketch)
    else:
        print('FAIL: Could not flash sketch %s' % sketch)
        return  {'success':0,'sn':None}
    time.sleep(2.0)
    x = find_arduino()
    if x['serial'] is not None:
        print('###################################################################')
        print('SUCCESS: Firmware flash complete')
        return {'success': 1, 'sn': x['serial']}
    else:
        print('###################################################################')
        print('Failure: could not find board SN')
        return {'success': 0, 'sn': x['serial']}


# ######################## RDK Tools ##########################################
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

def add_arduino_udev_line_etc(device_name, serial_no):
    f = open('/etc/udev/rules.d/95-hello-arduino.rules', 'r')
    x = f.readlines()
    x_out = ''
    overwrite = False
    uline = 'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="804d",MODE:="0666", ATTRS{serial}=="' + serial_no + '", SYMLINK+="' + device_name + '", ENV{ID_MM_DEVICE_IGNORE}="1"\n'
    for xx in x:
        if xx.find(device_name) > 0 and xx[0] != '#':
            overwrite = True
            x_out = x_out + uline
            print('Overwriting existing entry...')
        else:
            x_out = x_out + xx
    if not overwrite:
        print('Creating new entry...')
        x_out = x_out + uline
    f.close()
    f = open( '/tmp/95-hello-arduino.rules', 'w')
    f.write(x_out)
    f.close()
    os.system('sudo cp /tmp/95-hello-arduino.rules /etc/udev/rules.d')

def add_arduino_udev_line(device_name, serial_no,fleet_dir):
    f = open(fleet_dir+'udev/95-hello-arduino.rules', 'r')
    x = f.readlines()
    x_out = ''
    overwrite=False
    uline = 'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="804d",MODE:="0666", ATTRS{serial}=="'+serial_no + '", SYMLINK+="'+device_name+'", ENV{ID_MM_DEVICE_IGNORE}="1"\n'
    for xx in x:
        if xx.find(device_name) > 0 and xx[0]!='#':
            overwrite=True
            x_out = x_out + uline
            print('Overwriting existing entry...')
        else:
            x_out = x_out + xx
    if not overwrite:
        print('Creating new entry...')
        x_out = x_out + uline
    f.close()
    f = open(fleet_dir+'udev/95-hello-arduino.rules', 'w')
    f.write(x_out)
    f.close()

def add_ftdi_udev_line(device_name, serial_no,fleet_dir):
    f = open(fleet_dir+'udev/99-hello-dynamixel.rules', 'r')
    x = f.readlines()
    x_out = ''
    overwrite=False
    uline = 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTR{device/latency_timer}="1", ATTRS{serial}=="'+serial_no+'", SYMLINK+="'+device_name+'"'
    for xx in x:
        if xx.find(device_name) > 0 and xx[0]!='#':
            overwrite=True
            x_out = x_out + uline +'\n'
            print('Overwriting existing entry...')
        else:
            x_out = x_out + xx
    if not overwrite:
        print('Creating new entry...')
        x_out = x_out + uline + '\n'
    f.close()
    f = open(fleet_dir+'udev/99-hello-dynamixel.rules', 'w')
    f.write(x_out)
    f.close()

def assign_arduino_to_robot(device_name,is_stepper=False,robot_sn=None):
    """
    This expects only a single arduino device on the bus
    Tie the uC serial number to device_name under udev
    Also update the YAML with the serial number if a stepper
    The YAML writing requres the HELLO_FLEET_ID to be set in advance
    """
    if robot_sn is None:
       fleet_dir=hello_utils.get_fleet_directory()
    else:
        fleet_dir=os.environ['HELLO_FLEET_PATH']+'/'+robot_sn+'/'
    a = find_arduinos()
    if len(a) != 1:
        print('Error: Only one Arduino should be on the bus')
    else:
        sn=str(a[0].serial_number)
        add_arduino_udev_line(device_name,sn,fleet_dir)
        if is_stepper:
            print('Setting serial number in YAML for %s to %s'%(device_name,sn))
            d = stretch_body.device.Device(req_params=False)
            d.write_configuration_param_to_YAML(device_name+'.serial_no', sn,fleet_dir=fleet_dir)
        return  {'success': 1, 'sn': sn}
    return  {'success': 0, 'sn':None}

def assign_dynamixel_to_robot(device_name, robot_sn=None):
    """
    This expects only a single FTDI device on the bus
    Tie the FTDI serial number to device_name under udev
    """
    print('A',hello_utils.get_fleet_directory())
    print('B',os.environ['HELLO_FLEET_PATH'],robot_sn)
    if robot_sn is None:
        fleet_dir=hello_utils.get_fleet_directory()
    else:
        fleet_dir=os.environ['HELLO_FLEET_PATH']+'/'+robot_sn+'/'
    f=find_ftdis()
    if len(f) != 1:
        print('Error: Only one FTDI should be on the bus')
    else:
        sn=f[0].serial_number
        add_ftdi_udev_line(device_name,sn,fleet_dir)
        return  {'success': 1, 'sn': sn}
    return  {'success': 0, 'sn':None}
