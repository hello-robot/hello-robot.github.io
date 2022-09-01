#!/usr/bin/env python

# This script aims to calculate the effective wheel separation, to make more accurate base rotations.
# It's probably good to run the script multiple times with different commanded turning angles,
#   then use the average wheel separation parameter.
#
# This script assumes that Stretch can accurately drive straight by a commanded distance.
#   To test:
#     robot.translate_by(enter_distance_here_m)
#     robot.push_command()
#     [measure actual distance traveled]
#   If not, then the wheel diameter parameters may also need to be adjusted.
# Given this assumption, we can compute the effective wheel separation:
#   As Stretch is rotated, it will estimate how much it has rotated by measuring the distance each wheel has traveled:
#     theta_rad_reported = 2 * arc_length_encoders / wheel_separation_param     # using circle_arc = (pi * D) * (theta / 2pi) where D is the wheel separation
#   Since we assumed it computes wheel distance well, we assume arc_length_encoders is accurate.
#   If we rotate Stretch by a known angle, we can compute the effective wheel separation:
#     wheel_separation_actual = 2 * arc_length_encoders / theta_rad_actual
#     wheel_separation_actual = 2 * (theta_rad_reported * wheel_separation_param / 2) / theta_rad_actual
#     wheel_separation_actual = wheel_separation_param * (theta_rad_reported / theta_rad_actual)
#   So we basically just need to rotate Stretch by a known angle and then see how far it thinks it rotated.
#     The farther we rotate, the more accurate the linear interpolation may be (the less our manual inaccuracies will matter).
#     To avoid manually rotating the entire way though, we'll let Stretch take its best shot and then we'll just manually correct it.
#
# Created 2021-05-18 by Joseph DelPreto
# Last modified 2021-05-23 by Joseph DelPreto, based on forum discussion with Aaron Edsinger
# See https://forum.hello-robot.com/t/172 for more information
from future.builtins import input
import stretch_body.hello_utils as hello_utils
import stretch_body.robot
from math import radians, degrees, atan2, sin, cos
import time
import click

hello_utils.print_stretch_re_use()

import argparse

parser=argparse.ArgumentParser(description='Calibrate the wheel_separation_m parameter for the base to ensure accurate rotations.')
args = parser.parse_args()

# Helper functions
def runstop_stop():
  robot.pimu.runstop_event_trigger()
  robot.push_command()
  time.sleep(1)

def runstop_reset():
  robot.pimu.runstop_event_reset()
  robot.push_command()
  time.sleep(1)

def get_current_theta_rad():
  robot_status = robot.get_status()
  return robot_status['base']['theta']

def do_spin(robot, deg):
    wrapToPi = lambda theta_rad: atan2(sin(theta_rad), cos(theta_rad))
    print('############ TEST %f DEGREES ###############'%deg)
    print('')
    print('')

    runstop_stop()
    print('')
    print('Manually rotate Stretch so the front of its base (the bottom metal edge)')
    print('is aligned with a starting line on the floor.')
    input('  Press Enter when done ')
    runstop_reset()

    time.sleep(1) # let the reset take effect
    # Get Stretch's estimate of the current position.
    theta_startPosition_rad = get_current_theta_rad()

    # Rotate by a fixed amount.
    turn_command_rad = radians(deg)
    turn_command_rads = radians(30)
    turn_command_radss = radians(10)
    robot.base.rotate_by(turn_command_rad, v_r=turn_command_rads, a_r=turn_command_radss)
    robot.push_command()
    time.sleep(abs(turn_command_rad) / turn_command_rads + 2 * turn_command_rads / turn_command_radss)  # a slight overestimate of the duration

    time.sleep(2) # make sure the robot is settled

    # Get Stretch's estimate of the current position.
    theta_preManualRotation_rad = get_current_theta_rad()
    runstop_stop()
    print('')
    print('Stretch thinks it is at the target angle!')
    print('Now manually rotate Stretch so the front of its base (the bottom metal edge)')
    print('is actually aligned with the target angle (%g degrees).' % degrees(turn_command_rad))
    input('  Press Enter when done ')
    runstop_reset()

    # Get Stretch's estimate of the current position
    theta_postManualRotation_rad = get_current_theta_rad()
    turn_reported_rad = theta_postManualRotation_rad - theta_startPosition_rad
    # Adjust it to be within [-180, 180] of the commanded angle (since we may command more than 360 degrees but Stretch's angle wraps around)
    turn_reported_rad = turn_command_rad - wrapToPi(turn_command_rad - turn_reported_rad)
    # Compute the effective wheel separation
    wheel_separation_param_m = robot.base.params['wheel_separation_m']
    wheel_separation_actual_m = wheel_separation_param_m * (turn_reported_rad / turn_command_rad)

    # Print the results
    print('')
    print('')
    print('---------------------------------------------------')
    print('Actual rotation           : %6.1f deg' % degrees(turn_command_rad))
    print('Stretch thought it rotated: %6.1f deg' % degrees(turn_reported_rad))
    print('  ... so it was off by    : %6.1f deg' % degrees(wrapToPi(turn_reported_rad - turn_command_rad)))
    print('')
    print('Factory wheel separation parameter : %6.2f mm' % (315.4))
    print('Previous wheel separation parameter: %6.2f mm' % (wheel_separation_param_m * 1000))
    print('Computed effective wheel separation: %6.2f mm' % (wheel_separation_actual_m * 1000))
    print('---------------------------------------------------')
    print('')
    print('')
    return wheel_separation_actual_m

robot = stretch_body.robot.Robot()
robot.startup()
d0=do_spin(robot,360)
d1=do_spin(robot,720)
d2=do_spin(robot,-360)
d3=do_spin(robot,-720)

d_avg=(d0+d1+d2+d3)/4.0
print('Final result: wheel_separation_m of %6.4f'%d_avg)
if click.confirm('Would you like to save the result to configuration YAML?'):
    robot.write_configuration_param_to_YAML('base.wheel_separation_m', d_avg)
