#!/usr/bin/env python
from __future__ import print_function
from future.builtins import input
import math
import time
import argparse
import stretch_body.hello_utils as hu
import stretch_body.wacc as wacc

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate gravity scalar for the Wacc accelerometer')
args=parser.parse_args()

print('Calibrating Wacc. Ensure arm is retracted and level to ground')
input('Hit enter when ready')

w=wacc.Wacc()
w.startup()
if w.hw_valid:
    w.params['config']['accel_gravity_scale'] = 1.0
    # Calibrate accel
    cum=0.0
    for i in range(100):
        w.pull_status()
        z=math.sqrt(w.status['ax']**2+w.status['ay']**2+w.status['az']**2)
        print('Itr',i,'Val',z)
        cum=cum+z
        time.sleep(0.05)
    w.stop()
    cum=cum/100.0
    print('Got a average value of',cum)
    s=9.80665/cum
    accel_gravity_scale_max= 1.1
    accel_gravity_scale_min= 0.9
    if s>accel_gravity_scale_min and s<accel_gravity_scale_max:
        print('Gravity scalar of %f within bounds of %f to %f'%(s,accel_gravity_scale_min ,accel_gravity_scale_max ))
        print('Writing yaml...')
        w.write_configuration_param_to_YAML('wacc.config.accel_gravity_scale', s)
    else:
        print('Gravity scalar of %f outside bounds of %f to %f' % (s, accel_gravity_scale_min,accel_gravity_scale_max))
    w.stop()
