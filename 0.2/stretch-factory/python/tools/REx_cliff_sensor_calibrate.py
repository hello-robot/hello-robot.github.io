#!/usr/bin/env python
from future.builtins import input
import stretch_body.pimu as pimu
import time
import argparse

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Zero the cliff sensors to the floor surface')
args = parser.parse_args()

print('Ensure cliff sensors are not obstructed and base is on a flat surface')
input('Hit enter when ready')

p = pimu.Pimu()
old_zero=p.config['cliff_zero'][:]
p.config['cliff_zero'][0]=0
p.config['cliff_zero'][1]=0
p.config['cliff_zero'][2]=0
p.config['cliff_zero'][3]=0

p.startup()
p.pull_status()


#Calibrate cliff
cum=[0,0,0,0]
for k in range(100):
    p.pull_status()
    for i in range(4):
        cum[i]=cum[i]+p.status['cliff_range'][i]
    print('Itr',k,'Val',p.status['cliff_range'])
    time.sleep(0.05)
p.stop()
cum=[cum[0]/100.0,cum[1]/100.0,cum[2]/100.0,cum[3]/100.0]

#Taken from Hank QC
cliff_zero_min= 450
cliff_zero_max= 700
pass_test=1
print('Got cliff zeros of: ',cum)

if  not(cum[0]>cliff_zero_min and cum[0]<=cliff_zero_max):
    print('Cliff-zero-0 out of range')
    pass_test=0
if  not(cum[1]>cliff_zero_min and cum[1]<=cliff_zero_max):
    print('Cliff-zero-1 out of range')
    pass_test=0
if  not(cum[2]>cliff_zero_min and cum[2]<=cliff_zero_max):
    print('Cliff-zero-2 out of range')
    pass_test=0
if  not(cum[3]>cliff_zero_min and cum[3]<=cliff_zero_max):
    print('Cliff-zero-3 out of range')
    pass_test=0

if pass_test:
    print('Calibration passed. Storing to YAML...')
    for i in range(4):
        p.params['config']['cliff_zero'][i]=cum[i]
    p.write_configuration_param_to_YAML('pimu.config.cliff_zero',cum)
    p.stop()
else:
    print('Calibration failed...')
    p.stop()
