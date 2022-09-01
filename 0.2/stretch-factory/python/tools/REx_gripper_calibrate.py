#!/usr/bin/env python
from future.builtins import input
import stretch_body.stretch_gripper as gripper
import time
import stretch_body.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibate the griper range and zero')
args=parser.parse_args()


g=gripper.StretchGripper()
if not g.startup():
    exit(1)

#Good starting point
g.params['zero_t']=4000
g.params['range_t']=[0,100000]
#Reset soft motion limits as well
if g.params['flip_encoder_polarity']:
    wr_max = g.ticks_to_world_rad(g.params['range_t'][0])
    wr_min = g.ticks_to_world_rad(g.params['range_t'][1])
else:
    wr_max = g.ticks_to_world_rad(g.params['range_t'][1])
    wr_min = g.ticks_to_world_rad(g.params['range_t'][0])
g.soft_motion_limits = {'collision': [None, None], 'user': [None, None], 'hard': [wr_min, wr_max],
                           'current': [wr_min, wr_max]}

input('Hit enter to find zero')
g.home()
print('---------------------------------------------------')
print('Enter 1 to open fingers. Enter 2 to close fingers. Enter 3 when the fingertips are just barely not touching.')
z_done=False
while not z_done:
    x = input()
    if x=='1':
        g.move_by(5.0)
    elif x=='2':
        g.move_by(-5.0)
    elif x=='3':
        g.pull_status()
        print('Setting zero at:',g.status['pos_ticks'])
        g.params['zero_t']=g.status['pos_ticks']
        z_done=True
print('---------------------------------------------------')
print('Enter 1 to open fingers. Enter 2 to close fingers. Enter 3 when the fingertips are fully open, ')
print('and no further opening motion is possible')
z_done=False
while not z_done:
    x = input()
    if x == '1':
        g.move_by(5.0)
    elif x == '2':
        g.move_by(-5.0)
    elif x == '3':
        g.pull_status()
        print('Setting open at:',g.status['pos_ticks'])
        g.params['range_t']=[0,g.status['pos_ticks']]
        z_done=True


input('Hit enter to close')
g.move_to(-100)
input('Hit enter to open')
g.move_to(50.0)
input('Hit enter to go to zero')
g.move_to(0.0)
time.sleep(4.0)
g.stop()

x=input('Save calibration [y]?')
if x=='y' or x=='Y' or x=='':
    g.write_configuration_param_to_YAML('stretch_gripper.range_t', g.params['range_t'])
    g.write_configuration_param_to_YAML('stretch_gripper.zero_t', g.params['zero_t'])