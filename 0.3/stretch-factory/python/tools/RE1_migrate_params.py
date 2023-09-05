#!/usr/bin/env python
import argparse
import stretch_factory.param_mgmt as param_mgmt
import os
import stretch_body.hello_utils as hello_utils
from os.path import exists
import click

def dir_path(string):
    if os.path.isdir(string):
        return string
    else:
        print('Invalid path: %s'%string)
        exit()

parser = argparse.ArgumentParser(description='Migrate Stretch parameter data to latest format')
parser.add_argument("--no_prompt", help="Don't use prompts", action="store_true")
parser.add_argument("--diff", help="Check differences of proposed migrated params to current params", action="store_true")
parser.add_argument("--path", help="Path to robot parameters (if not $HELLO_FLEET_PATH/$HELLO_FLEET_ID)", type=dir_path)
args = parser.parse_args()

if args.path is None:
    fleet_dir=hello_utils.get_fleet_directory()
else:
    fleet_dir=args.path
fleet_dir=fleet_dir.rstrip('/')
fleet_path=fleet_dir[:-17]
fleet_id=fleet_dir[-16:]

def cleanup_generated_files():
    # Cleanup
    if exists(hello_utils.get_fleet_directory() + 'stretch_user_params.yaml'):
        os.system('rm %s' % (hello_utils.get_fleet_directory() + 'stretch_user_params.yaml'))
    if exists(hello_utils.get_fleet_directory() + 'stretch_configuration_params.yaml'):
        os.system('rm %s' % (hello_utils.get_fleet_directory() + 'stretch_configuration_params.yaml'))

# Point to the data to be migrated
hello_utils.set_fleet_directory(fleet_path, fleet_id)
click.secho('############## Migrating  %s ###############'%fleet_id,fg="green")
print('Checking parameters  at: %s'%fleet_dir)
print('-------------------------------------')

if exists(hello_utils.get_fleet_directory()+'stretch_user_params.yaml') and exists(hello_utils.get_fleet_directory()+'stretch_configuration_params.yaml'):
    click.secho('Parameter format is up to date. No migration is required.', fg="green")
    exit(0)
if not exists(hello_utils.get_fleet_directory()+'stretch_re1_user_params.yaml') or not exists(hello_utils.get_fleet_directory()+'stretch_re1_factory_params.yaml'):
    click.secho('Robot paramters not found. Unable to migrate. Please contact Hello Robot support for more information.', fg="red")
    exit(1)

if args.diff or click.confirm('Migration is required for robot %s. Proceed?'%fleet_id):
    if 1: #try:
        drop_user_params = []#['factory_params', 'tool_params']
        O, U, R=param_mgmt.migrate_params_RE1V0(fleet_path, fleet_id,drop_user_params)
        if O==None:#Failed
            click.secho('Robot paramters corrupted. Unable to migrate. Please contact Hello Robot support for more information.', fg="red")
            exit(1)
        print('Migration complete. Starting validation...')

        #Read in the new parameter data
        import stretch_body.robot_params
        (UU, RR) = stretch_body.robot_params.RobotParams().get_params()

        #These parameters are whitelisted (OK'd) to be added/dropped/changed during migration after manual review of the migration process
        #of all robots up to an including Louis

        #Parameters that have been introduced since the customer yaml was generated
        #Disable this. It is OK to introduce new parameters over time.
        #added_whitelist=['head_pan.stall_backoff']

        #These are parameters that have been deperecated so we're OK dropping them from the new yaml
        dropped_whitelist=['factory_params','tool_params', 'robot.use_arm', 'robot.use_wacc', 'robot.use_lift', 'robot.use_end_of_arm', 'robot.use_head', 'robot.use_pimu','robot.use_base' ]

        #We are OK changing these parameters on the customer during migration as we need to take back control of the ability to update them via Pip
        change_whitelist=['head_tilt.pid','pimu.config.bump_thresh','base.contact_thresh_max_N','hello-motor-lift.gains.i_safety_feedforward','logging.handlers.file_handler.filename',
                          'pimu.config.stop_at_low_voltage']

        # Now check for differences

        #added_warnings = param_mgmt.param_added_check(RR, R, 0, 'NewParams', 'OldParams',whitelist=added_whitelist)
        dropped_warnings = param_mgmt.param_dropped_check(RR, R, 0, 'NewParams', 'OldParams',whitelist=dropped_whitelist)
        change_warnings = param_mgmt.param_change_check(RR, R, 0, 'NewParams', 'OldParams',whitelist=change_whitelist)

        if dropped_warnings+change_warnings>0:
            color='red'
            ret=1
        else:
            color='green'
            ret=0
        click.secho('Validation check: Dropped %d, Changed %d' % (dropped_warnings, change_warnings),fg=color)
        click.secho('Validation check should report 0 warnings. Reported total of %d' % (dropped_warnings+change_warnings),fg=color)

        if args.diff:
            cleanup_generated_files()
            exit(ret)
        else:
            if click.confirm('Use new parameters for robot %s?'%fleet_id):
                etc_dir='/etc/hello-robot/'+fleet_id+'/'

                uf= hello_utils.get_fleet_directory() + 'stretch_re1_user_params.yaml'
                uf_backup='stretch_re1_user_params.migration_backup.%s.yaml'%(hello_utils.create_time_string())
                print('Backing up %s to %s' % (uf, uf_backup))
                os.system('cp %s %s' % (uf, hello_utils.get_fleet_directory() + uf_backup))
                os.system('sudo cp %s %s'%(uf, etc_dir+uf_backup))
                os.system('rm %s'%uf)
                os.system('sudo rm %s' % etc_dir + 'stretch_re1_user_params.yaml')
                os.system('sudo cp %s %s'%(hello_utils.get_fleet_directory() + 'stretch_user_params.yaml', etc_dir))

                ff = hello_utils.get_fleet_directory() + 'stretch_re1_factory_params.yaml'
                ff_backup = 'stretch_re1_factory_params.migration_backup.%s.yaml' % (hello_utils.create_time_string())
                print('Backing up %s to %s' % (ff, ff_backup))
                os.system('cp %s %s' % (ff, hello_utils.get_fleet_directory() + ff_backup))
                os.system('sudo cp %s %s' % (ff, etc_dir + ff_backup))
                os.system('rm %s' % ff)
                os.system('sudo rm %s' % etc_dir + 'stretch_re1_factory_params.yaml')
                os.system('sudo cp %s %s' % (hello_utils.get_fleet_directory() + 'stretch_configuration_params.yaml', etc_dir))

                click.secho("Robot %s now configured to use latest parameter format"%fleet_id,fg='green')

            else:
                cleanup_generated_files()
            exit(0)
    # except:
    #     print('Exception. Cleaning up')
    #     cleanup_generated_files()
    #     exit(1)
else:
    exit(0)
