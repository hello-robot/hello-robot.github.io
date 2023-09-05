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

parser = argparse.ArgumentParser(description='Migrate Stretch contact force parameters to effort_pct model')
parser.add_argument("--no_prompt", help="Don't use prompts", action="store_true")
parser.add_argument("--path", help="Path to robot parameters (if not $HELLO_FLEET_PATH/$HELLO_FLEET_ID)", type=dir_path)
args = parser.parse_args()

if args.path is None:
    fleet_dir=hello_utils.get_fleet_directory()
else:
    fleet_dir=args.path
fleet_dir=fleet_dir.rstrip('/')
fleet_path=fleet_dir[:-17]
fleet_id=fleet_dir[-16:]


# Point to the data to be migrated
hello_utils.set_fleet_directory(fleet_path, fleet_id)
click.secho('############## Migrating  %s ###############'%fleet_id,fg="green")
print('Checking parameters  at: %s'%fleet_dir)
print('-------------------------------------')

if not exists(hello_utils.get_fleet_directory()+'stretch_user_params.yaml') or not exists(hello_utils.get_fleet_directory()+'stretch_configuration_params.yaml'):
    click.secho('Parameter format is not up to date. Run RE1_migrate_params.py first', fg="red")
    exit(0)


if click.confirm('Attempting migration of contact data for robot %s. Proceed?'%fleet_id):
    etc_dir = '/etc/hello-robot/' + fleet_id + '/'

    uf = hello_utils.get_fleet_directory() + 'stretch_user_params.yaml'
    uf_backup = 'stretch_user_params.contact_migration_backup.%s.yaml' % (hello_utils.create_time_string())
    print('Backing up %s to %s' % (uf, uf_backup))
    os.system('cp %s %s' % (uf, hello_utils.get_fleet_directory() + uf_backup))
    os.system('sudo cp %s %s' % (uf, etc_dir + uf_backup))

    cf = hello_utils.get_fleet_directory() + 'stretch_configuration_params.yaml'
    cf_backup = 'stretch_configuration_params.contact_migration_backup.%s.yaml' % (hello_utils.create_time_string())
    print('Backing up %s to %s' % (cf, cf_backup))
    os.system('cp %s %s' % (cf, hello_utils.get_fleet_directory() + cf_backup))
    os.system('sudo cp %s %s' % (cf, etc_dir + cf_backup))

    param_mgmt.migrate_contact_params_RE1V0(fleet_path, fleet_id)

    click.secho("Robot %s configured to use effort_pct contact parameters" % fleet_id, fg='green')
