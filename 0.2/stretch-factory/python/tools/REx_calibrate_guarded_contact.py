#!/usr/bin/env python
from stretch_body.hello_utils import *
import stretch_body.scope
import argparse
import click

print_stretch_re_use()

parser = argparse.ArgumentParser(description='Calibrate the default guarded contacts for a joint.')
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--lift", help="Calibrate the lift joint", action="store_true")
group.add_argument("--arm", help="Calibrate the arm joint", action="store_true")
parser.add_argument("--ncycle", type=int,help="Number of sweeps to run [4]",default=4)
args = parser.parse_args()

contact_model='effort_pct'

if args.lift:
    import stretch_body.lift
    j=stretch_body.lift.Lift()
if args.arm:
    import stretch_body.arm
    j=stretch_body.arm.Arm()

if j.robot_params['robot']['model_name']=='RE1V0':
    print('This is tool is not yet supported for robot model RE1V0')
    exit(1)

if not j.startup(threaded=False):
    exit(1)

j.pull_status()
if not j.motor.status['pos_calibrated']:
    print('Joint not calibrated. Exiting.')
    exit(1)


if (j.name in j.user_params and 'contact_models' in j.user_params[j.name]) and ('effort_pct' in j.user_params[j.name]['contact_models']) \
        and ('contact_thresh_default' in j.user_params[j.name]['contact_models']['effort_pct']):
    click.secho('------------------------',fg="yellow")
    click.secho('NOTE: This tool updates contact_thresh_default for %s in stretch_configuration_params.yaml'%j.name.upper(),fg="yellow")
    click.secho('NOTE: Your stretch_user_params.yaml overrides contact_thresh_default for %s'%j.name.upper(),fg="yellow")
    click.secho('NOTE: As such, the updated calibration will not change the contact behavior unless you remove the user params.',fg="yellow")
click.secho('------------------------',fg="yellow")
click.secho('Joint %s will go through its full range-of-motion. Ensure workspace is collision free '%j.name.capitalize(),fg="yellow")
if click.confirm('Proceed?'):
    j.motor.disable_sync_mode()
    xpos_pos=j.params['range_m'][1]
    xpos_neg=j.params['range_m'][0]
    j.motor.disable_guarded_mode()
    j.push_command()
    j.move_to(xpos_neg)
    j.push_command()
    j.motor.wait_until_at_setpoint()

    log_dir = get_stretch_directory('log/')
    log_file_ts=create_time_string()
    log_file_prefix='calibrate_guarded_contact_{0}'.format(j.name)
    print('')
    print('------------------------------------')
    print('Starting data collection...')

    effort_pct_pos = [[], [], [], []]
    effort_pct_neg = [[], [], [], []]
    pos_out = [[], [], [], []]
    pos_in = [[], [], [], []]
    max_effort_pct_pos = 0
    min_effort_pct_neg = 0
    for i in range(args.ncycle):
        j.move_to(xpos_pos)
        j.push_command()
        time.sleep(0.25)
        j.pull_status()
        ts = time.time()
        while not j.motor.status['near_pos_setpoint'] and time.time() - ts < 15.0:
            time.sleep(0.1)
            j.pull_status()
            effort_pct_pos[i].append(j.motor.status['effort_pct'])
            pos_out[i].append(j.status['pos'])
        max_effort_pct_pos = max(max_effort_pct_pos, max(effort_pct_pos[i]))
        print('Positive Motion: Itr %d  Max %f (Pct)' % (i, max(effort_pct_pos[i])))
    
        j.move_to(xpos_neg)
        j.push_command()
        time.sleep(0.25)
        j.pull_status()
        ts = time.time()
        while not j.motor.status['near_pos_setpoint'] and time.time() - ts < 15.0:
            time.sleep(0.1)
            j.pull_status()
            effort_pct_neg[i].append(j.motor.status['effort_pct'])
            pos_in[i].append(j.status['pos'])
        print('Negative Motion: Itr %d Min %f (Pct)' % (i,  min(effort_pct_neg[i])))
        min_effort_pct_neg = min(min_effort_pct_neg, min(effort_pct_neg[i]))
    results = {'effort_pct_pos': effort_pct_pos, 'effort_pct_neg': effort_pct_neg, 'pos_out': pos_out, 'pos_in': pos_in, 'max_effort_pct_pos': max_effort_pct_pos,'min_effort_pct_neg': min_effort_pct_neg}
    print('Maximum effort_pct pos: %f'%max_effort_pct_pos)
    print("Minimum effort_pct neg: %f"%min_effort_pct_neg)
    print('--------------------------------------')
    print('')

    t = time.localtime()
    log_filename=log_dir+log_file_prefix+'_results_'+log_file_ts+'.log'
    print('Writing results log: %s'%log_filename)
    with open(log_filename, 'w') as yaml_file:
        yaml.dump(results, yaml_file)

    s = stretch_body.scope.Scope4(yrange=[-110, 110], title='Effort(%) Postive')
    s.draw_array_xy(pos_out[0], pos_out[1], pos_out[2], pos_out[3], effort_pct_pos[0], effort_pct_pos[1], effort_pct_pos[2], effort_pct_pos[3])
    img_filename=log_dir+log_file_prefix+'_pos_effort_pct_'+log_file_ts+'.png'
    print('Writing image log: %s' % img_filename)
    s.savefig(img_filename)

    s = stretch_body.scope.Scope4(yrange=[-110, 110], title='Effort(%) Negative')
    s.draw_array_xy(pos_in[0], pos_in[1], pos_in[2], pos_in[3], effort_pct_neg[0], effort_pct_neg[1], effort_pct_neg[2], effort_pct_neg[3])
    img_filename = log_dir + log_file_prefix + '_neg_effort_pct_' + log_file_ts + '.png'
    print('Writing image log: %s' % img_filename)
    s.savefig(img_filename)

    ocd_n = j.params['contact_models']['effort_pct']['contact_thresh_default'][0]
    ocd_p = j.params['contact_models']['effort_pct']['contact_thresh_default'][1]

    ncd_n = max(j.params['contact_models']['effort_pct']['contact_thresh_max'][0],min_effort_pct_neg - j.params['contact_models']['effort_pct']['contact_thresh_calibration_margin'])
    ncd_p = min(j.params['contact_models']['effort_pct']['contact_thresh_max'][1],max_effort_pct_pos +j.params['contact_models']['effort_pct']['contact_thresh_calibration_margin'])

    #Flag if within 20% of maximums
    if ncd_n<j.params['contact_models']['effort_pct']['contact_thresh_max'][0]*0.8:
        click.secho('Warning: Contact threshold of %f near minimum of %f. There may be a mechanical issue with the joint. Contact Hello Robot support'%
                    (ncd_n,j.params['contact_models']['effort_pct']['contact_thresh_max'][0]), fg="red")

    if ncd_p>j.params['contact_models']['effort_pct']['contact_thresh_max'][1]*0.8:
        click.secho('Warning: Contact threshold of %f near maximum of %f. There may be a mechanical issue with the joint. Contact Hello Robot support'%
                    (ncd_p,j.params['contact_models']['effort_pct']['contact_thresh_max'][1]), fg="red")

    click.secho('')
    click.secho('Prior contact defaults were:',fg="green")
    click.secho('----------------------------')
    click.secho('Positive direction:  %f (Effort)'%(ocd_p),fg="green")
    click.secho('Negative direction: %f (Effort)'%(ocd_n),fg="green")
    click.secho('')
    click.secho('New contact defaults are:',fg="green")
    click.secho('----------------------------')
    click.secho('Positive direction: %f (Effort)'%(ncd_p),fg="green")
    click.secho('Negative direction: %f (Effort)'%(ncd_n),fg="green")

    if click.confirm('Save results?'):
        j.write_configuration_param_to_YAML(j.name + '.contact_models.effort_pct.contact_thresh_default', [ncd_n,ncd_p])

