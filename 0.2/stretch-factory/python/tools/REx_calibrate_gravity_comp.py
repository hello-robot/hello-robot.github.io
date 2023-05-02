#!/usr/bin/env python3
from stretch_body.hello_utils import *
import stretch_body.scope
import argparse
import click

print_stretch_re_use()

parser = argparse.ArgumentParser(description='Recalibrate the gravity compensation for the lift (given a new payload, etc).')
args = parser.parse_args()

contact_model='effort_pct'


import stretch_body.lift
j=stretch_body.lift.Lift()

check_deprecated_contact_model_prismatic_joint(j, 'REx_calibrate_guarded_contacts.py', None, None, None, None)

if not j.startup(threaded=False):
    exit(1)

j.pull_status()
if not j.motor.status['pos_calibrated']:
    print('Joint not calibrated. Exiting.')
    exit(1)

if ('lift' in j.user_params and 'i_feedforward' in j.user_params['lift']) :
    click.secho('------------------------',fg="yellow")
    click.secho('NOTE: This tool updates lift.i_feedforward in your stretch_configuration_params.yaml',fg="yellow")
    click.secho('NOTE: However your stretch_user_params.yaml contains an overriding value for lift.i_feedforward',fg="yellow")
    click.secho('NOTE: As such, the updated calibration will not change the robot behavior unless you remove the user params.',fg="yellow")
    click.secho('------------------------',fg="yellow")

if ('hello-motor-lift' in j.user_params and 'gains' in j.user_params['hello-motor-lift'] and 'i_safety_feedforward' in j.user_params['hello-motor-lift']['gains']) :
    click.secho('------------------------',fg="yellow")
    click.secho('NOTE: This tool updates hello-motor-lift.gains.i_safety_feedforward in your stretch_configuration_params.yaml',fg="yellow")
    click.secho('NOTE: However your stretch_user_params.yaml contains an overriding value for hello-motor-lift.gains.i_safety_feedforward',fg="yellow")
    click.secho('NOTE: As such, the updated calibration will not change the robot behavior unless you remove the user params.',fg="yellow")
    click.secho('------------------------',fg="yellow")

if click.confirm('Measuring gravity compensation parameter. Proceed?'):

    log_dir = get_stretch_directory('log/')
    log_file_ts = create_time_string()
    log_file_prefix = 'calibrate_gravity_comp_{0}'.format(j.name)
    print('')
    print('------------------------------------')
    print('Starting data collection...')

    j.motor.disable_sync_mode()
    j.push_command()
    j.move_to(0.5)
    j.push_command()
    j.motor.wait_until_at_setpoint()

    j.move_to(0.6)
    j.push_command()
    j.motor.wait_until_at_setpoint()
    time.sleep(1.5)
    i_up=[]
    i_sum=0
    for n in range(10):
        j.pull_status()
        i=j.motor.status['current']
        print('Up direction %d: Current: %f'%(n,i))
        i_sum=i_sum+i
        i_up.append(i)
        time.sleep(0.5)

    j.move_to(0.4)
    j.push_command()
    j.motor.wait_until_at_setpoint()
    time.sleep(1.5)
    i_down=[]
    for n in range(10):
        j.pull_status()
        i = j.motor.status['current']
        print('Down direction %d: Current: %f' % (n+10, i))
        i_down.append(i)
        i_sum = i_sum + i
        time.sleep(0.5)

    i_avg=abs(i_sum/20.0)

    results = {'i_avg': i_avg, 'i_up':i_up, 'i_down':i_down }

    print('Prior i_feedforward of (A): %f' % j.params['i_feedforward'])
    print('Prior i_safety_feedforward of (A): %f'%j.motor.params['gains']['i_safety_feedforward'])
    print()
    click.secho('New gravity compensation of (A): %f for i_feedforward and i_feedforward'%i_avg,fg="green")
    print('')

    #Nominal gravity comp with standard gripper should be about 1.2A (~effort 30)
    #Allow for DexWrist plus payload, leaving some headroom for dynamics
    #So recommend a gravity comp below 60% max current
    thresh=abs(j.motor.effort_pct_to_current(60.0))
    if i_avg>thresh:
        click.secho('------------------------', fg="yellow")
        click.secho('WARNING: The proposed new gravity compensation value of %f exceeds acceptable bounds of %f'%(i_avg,thresh),fg="yellow")
        click.secho('WARNING: This may be due to an excessivly heavy payload on the arm', fg="yellow")
        click.secho('WARNING: Or due to electro-mechanical issues with the lift.', fg="yellow")
        click.secho('WARNING: Please contact Hello Robot support (support@hello-robot.com)', fg="yellow")
        exit(1)

    if click.confirm('Save results?'):
        t = time.localtime()
        log_filename = log_dir + log_file_prefix + '_results_' + log_file_ts + '.log'
        print('Writing results log: %s' % log_filename)
        with open(log_filename, 'w') as yaml_file:
            yaml.dump(results, yaml_file)
        j.write_configuration_param_to_YAML('lift.i_feedforward', i_avg,force_creation=True)
        j.write_configuration_param_to_YAML('hello-motor-lift.gains.i_safety_feedforward', i_avg,force_creation=True)

