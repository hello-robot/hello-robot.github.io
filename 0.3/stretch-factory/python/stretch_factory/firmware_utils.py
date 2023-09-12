#!/usr/bin/env python

import click
import os
from subprocess import Popen, PIPE
import stretch_body.stepper
import stretch_body.pimu
import stretch_body.wacc
import stretch_body.device
import time
import sys
import stretch_body.device
import stretch_body.hello_utils
import shlex
import stretch_factory.hello_device_utils as hdu

log_device = stretch_body.device.Device(req_params=False)

def user_msg_log(msg, user_display=True, fg=None, bg=None, bold=False):
    if user_display:
        click.secho(str(msg), fg=fg, bg=bg, bold=bold)
    log_device.logger.debug(str(msg))

def check_ubuntu_version():
    res = Popen(shlex.split('cat /etc/lsb-release | grep DISTRIB_RELEASE'), shell=False, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read().strip(b'\n')
    return res == b'DISTRIB_RELEASE=18.04'

def check_arduino_cli_install(no_prompts=False):
    target_version = b'0.31.0'  # 0.18.3'
    version = 'None'
    res = Popen(shlex.split('arduino-cli version'), shell=False, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()
    do_install = False
    if not (res[:11] == b'arduino-cli'):
        do_install = True
    else:
        version = res[res.find(b'Version:') + 9:res.find(b' Commit')]
        if version != target_version:
            do_install = True
    if do_install:
        click.secho('WARNING:---------------------------------------------------------------------------------',
                    fg="yellow", bold=True)
        click.secho('WARNING: Compatible version of arduino_cli not installed. ', fg="yellow", bold=True)
        click.secho('Requires version %s. Installed version of %s' % (target_version, version))
        if no_prompts or click.confirm('Install now?'):
            os.system(
                'curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin/ sh -s %s' % target_version.decode(
                    'utf-8'))
            os.system('arduino-cli config init --overwrite')
            os.system('arduino-cli core install arduino:samd@1.6.21')
            os.system("sed -i -e 's#Arduino#repos/stretch_firmware/arduino#g' ~/.arduino15/arduino-cli.yaml")
            return True
        else:
            return False
    return True



def get_sketch_name(device_name):
    if device_name=='hello-motor-left-wheel' or device_name=='hello-motor-right-wheel' or device_name=='hello-motor-arm' or device_name=='hello-motor-lift':
        return 'hello_stepper'
    if device_name == 'hello-wacc':
        return 'hello_wacc'
    if device_name == 'hello-pimu':
        return 'hello_pimu'

def exec_process(cmdline, silent, input=None, **kwargs):
    """Execute a subprocess and returns the returncode, stdout buffer and stderr buffer.
       Optionally prints stdout and stderr while running."""
    try:
        sub = Popen(cmdline, stdin=PIPE, stdout=PIPE, stderr=PIPE,**kwargs)
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
        raise RuntimeError('Got return value %d while executing "%s", stderr output was:\n%s' % (
        returncode, " ".join(cmdline), stderr.rstrip(b"\n")))
    return stdout


def is_device_present(device_name):
    try:
        exec_process(['ls', '/dev/'+device_name], True)
        return True
    except RuntimeError as e:
        return False

def wait_on_device(device_name,timeout=10.0):
    #Wait for device to appear on bus for timeout seconds
    print('Waiting for device %s to return to bus.'%device_name)
    ts=time.time()
    itr=0
    while(time.time()-ts<timeout):
        if is_device_present(device_name):
            return True
        itr=itr+1
        if itr % 5 == 0:
            sys.stdout.write('.')
            sys.stdout.flush()
        time.sleep(0.1)
    return False

def get_port_name(device_name):
    try:
        port_name = Popen(shlex.split("ls -l /dev/" + device_name), shell=False, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read().strip().split()[-1]
        if not type(port_name)==str:
            port_name=port_name.decode('utf-8')
        return port_name
    except IndexError:
        return None

def does_stepper_have_encoder_calibration_YAML(device_name):
    d=stretch_body.device.Device(req_params=False)
    sn = d.robot_params[device_name]['serial_no']
    fn = 'calibration_steppers/' + device_name + '_' + sn + '.yaml'
    enc_data = stretch_body.hello_utils.read_fleet_yaml(fn)
    return len(enc_data)!=0

def get_device_protocols(device_name):
    #return list like ['p0','p1']
    s=get_sketch_name(device_name)
    if s == 'hello_wacc':
        import stretch_body.wacc
        dd = stretch_body.wacc.Wacc()
        return list(dd.supported_protocols.keys())
    elif s == 'hello_pimu':
        import stretch_body.pimu
        dd = stretch_body.pimu.Pimu()
        return list(dd.supported_protocols.keys())
    elif s == 'hello_stepper':
        import stretch_body.stepper
        dd = stretch_body.stepper.Stepper('/dev/'+device_name)
        return list(dd.supported_protocols.keys())
    return []

