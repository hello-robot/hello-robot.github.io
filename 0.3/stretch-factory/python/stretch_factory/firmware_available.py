import click
import os
import git
import sys
from stretch_factory.firmware_version import FirmwareVersion

class FirmwareAvailable():
    """
    Determine what firmware is available on GIT
    """
    def __init__(self, use_device):
        self.use_device = use_device #True/False dict for each device name
        self.repo = None
        self.repo_path = None
        self.versions = {}
        for d in self.use_device:
            if self.use_device[d]:
                self.versions[d] = []  # List of available versions for that device
        self.__clone_firmware_repo()
        self.__get_available_firmware_versions()

    def __clone_firmware_repo(self):
        print('Collecting information...', end='')
        self.repo_path = '/tmp/stretch_firmware_update'
        if not os.path.isdir(self.repo_path):
            # print('Cloning latest version of Stretch Firmware to %s'% self.repo_path)
            try:
                git.Repo.clone_from('https://github.com/hello-robot/stretch_firmware', self.repo_path)
            except git.GitCommandError as e:
                if "could not resolve host" in e.stderr.lower():
                    print("ERROR: Unable to connect to Github. Check internet connection?", file=sys.stderr)
                else:
                    print("ERROR: Unable to clone stretch_firmware from Github into /tmp/stretch_firmware_update.",
                          file=sys.stderr)
                sys.exit(1)

        sys.stdout.write('.')
        sys.stdout.flush()
        self.repo = git.Repo(self.repo_path)
        os.chdir(self.repo_path)

        sys.stdout.write('.')
        sys.stdout.flush()
        os.system('git checkout master >/dev/null 2>&1')

        sys.stdout.write('.')
        sys.stdout.flush()
        os.system('git fetch --tags >/dev/null 2>&1 ')

        sys.stdout.write('.')
        sys.stdout.flush()
        os.system('git pull >/dev/null 2>&1 ')
        sys.stdout.write('.')
        sys.stdout.flush()
        print('\n')

    def pretty_print(self):
        click.secho(' Currently Tagged Versions of Stretch Firmware on Master Branch '.center(110, '#'), fg="cyan",
                    bold=True)
        for device_name in self.versions:#.keys():
            click.secho('---- %s ----' % device_name.upper(), fg="white", bold=True)
            for v in self.versions[device_name]:
                print(v)

    # python -m pip install gitpython
    # https://www.devdungeon.com/content/working-git-repositories-python
    def __get_available_firmware_versions(self):
        if self.repo is None:
            return
        for t in self.repo.tags:
            v = FirmwareVersion(t.name)
            if v.valid:
                for device_name in self.versions:
                    if (v.device == 'Stepper' and device_name in ['hello-motor-lift', 'hello-motor-arm',
                                                                  'hello-motor-left-wheel',
                                                                  'hello-motor-right-wheel']) or \
                            (v.device == 'Wacc' and device_name == 'hello-wacc') or \
                            (v.device == 'Pimu' and device_name == 'hello-pimu'):
                        self.versions[device_name].append(v)

    def get_most_recent_version(self, device_name, supported_protocols):
        """
        For the device and supported protocol versions (eg, '['p0','p1']'), return the most recent version (type FirmwareVersion)
        """
        if len(self.versions[device_name]) == 0:
            return None
        recent = None
        if supported_protocols is not None:
            s = [int(x[1:]) for x in supported_protocols]
        else:
            class Everything(object):
                def __contains__(self, other):
                    return True
            s = Everything()
        supported_versions = []
        for v in self.versions[device_name]:
            if v.protocol in s:
                supported_versions.append(v)
        for sv in supported_versions:
            if recent is None or sv > recent:
                recent = sv
        return recent

    def get_remote_branches(self):
        branches = []
        print('Collecting information...')
        for ref in self.repo.git.branch('-r').split('\n'):
            sys.stdout.write('.')
            sys.stdout.flush()
            branches.append(ref)
        print('\n')
        branches = [b.strip(' ') for b in branches if b.find('HEAD') == -1]
        branches.remove('origin/master')  # Move master to position 0
        branches = ['origin/master'] + branches
        return branches

