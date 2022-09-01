#!/usr/bin/env python


import subprocess
import sys


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

def find_ftdi_sn():
    try:
        x=exec_process([b'sudo',b'lsusb', b'-d', b'0403:6001',b'-v'], True).split(b'\n')
        for ln in x:
            if ln.find(b'iSerial')>=0:
                sn=ln.split(b' ')[-1]
                print(sn)
    except RuntimeError:
        print('FTDI device not found')

def find_arduino_sn():
    try:
        x = exec_process([b'sudo', b'lsusb', b'-d', b'2341:804d', b'-v'], True).split(b'\n')
        for ln in x:
            if ln.find(b'iSerial') >= 0:
                sn = ln.split(b' ')[-1]
                print(sn)
    except RuntimeError:
        print('Arduino device not found')



print('---FTDI devices---')
find_ftdi_sn()

print('---Arduino devices---')
find_arduino_sn()