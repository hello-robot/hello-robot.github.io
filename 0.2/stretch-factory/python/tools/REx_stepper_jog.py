#!/usr/bin/env python
import sys
import stretch_body.stepper as stepper
import stretch_body.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Menu interface to a Stepper controller')
parser.add_argument('stepper_name', metavar='stepper_name', type=str, nargs=1,help='Provide the stepper name e.g.: hello-motor-lift')
args=parser.parse_args()

motor = stepper.Stepper('/dev/'+args.stepper_name[0])
if not motor.startup():
    exit(1)

motor.disable_sync_mode()
motor.disable_runstop()
motor.push_command()

def menu_top():
    print('-----------')
    print('ms: mode safety')
    print('mf: mode freewheel')
    print('mb: mode hold')
    print('mq: mode pos pid')
    print('mr: mode vel pid')
    print('mx: mode pos traj')
    print('mc: mode pos traj incr')
    print('mv: mode vel traj')
    print('mi: mode current')
    print('x <val>: set x_des')
    print('v <val>: set v_des')
    print('a <val>: set a_des')
    print('s <val>: set stiffness')
    print('f <val>: set feedforward')
    print('y <val>: set safety_feedforward')
    print('p <val>: set i_contact_pos')
    print('n <val>: set i_contact_neg')
    print('i <val>: set current')
    print('z <val>: mark position')
    print('g: set gain')
    print('r: reset board')

def menu_gains():
    print('-----------')
    for k in motor.gains.keys():
        print(k + '     <val> [' + str(motor.gains[k])+']')

def set_gains():
    x = sys.stdin.readline()
    if len(x)>4:
        print(x, len(x))
        g=x[:x.find(' ')]
        v=float(x[x.find(' '):])
        for k in motor.gains.keys():
            if g==k:
                motor.gains[k]=v
        motor.set_gains(motor.gains)

def set_mode(x):
    if x[1]=='s':
        motor.enable_safety()
    if x[1]=='f':
        motor.enable_freewheel()
    if x[1] == 'b':
        motor.enable_hold()
    if x[1] == 'i':
        motor.enable_current()
    if x[1] == 'q':
        motor.enable_pos_pid()
    if x[1] == 'r':
        motor.enable_vel_pid()
    if x[1] == 'x':
        motor.enable_pos_traj()
    if x[1] == 'c':
        motor.enable_pos_traj_incr()
    if x[1] == 'v':
        motor.enable_vel_traj()


def step_interaction():
    global sp, ff, st
    menu_top()
    x=sys.stdin.readline()
    motor.pull_status()
    if len(x)>1:
        if x[0]=='z':
            motor.enable_safety()
            motor.push_command()
            motor.mark_position(float(x[1:]))
            motor.push_command()
            motor.pull_status()
            print('Position marked. Motor safety on.')
            print('At', motor.status['pos'])
        if x[0]=='m':
            set_mode(x)
        if x[0]=='x':
            sp=float(x[1:])
            motor.set_command(x_des=sp)
        if x[0]=='v':
            sp=float(x[1:])
            motor.set_command(v_des=sp)
        if x[0]=='a':
            sp=float(x[1:])
            motor.set_command(a_des=sp)
        if x[0]=='s':
            st=float(x[1:])
            motor.set_command(stiffness=st)
        if x[0]=='i':
            ii=float(x[1:])
            motor.set_command(mode=motor.MODE_CURRENT,i_des=ii)
        if x[0]=='f':
            ff=float(x[1:])
            motor.set_command(i_feedforward=ff)
        if x[0]=='y':
            ff=float(x[1:])
            motor.gains['i_safety_feedforward']=(max(0,min(5.0,ff)))
            motor.set_gains(motor.gains)
        if x[0]=='p':
            ff=float(x[1:])
            motor.set_command(i_contact_pos=ff)
        if x[0]=='n':
            ff=float(x[1:])
            motor.set_command(i_contact_neg=ff)
        if x[0]=='g':
            menu_gains()
            set_gains()
        if x[0]=='r':
            print('Resetting Board. Restart process...')
            motor.board_reset()
            motor.push_command()
    else:
        motor.pretty_print()
    motor.push_command()

try:
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit):
    motor.stop()

