#!/usr/bin/env python

import click
import os
from subprocess import Popen, PIPE
import git
import stretch_body.stepper
import stretch_body.pimu
import stretch_body.wacc
import yaml
import time
import sys
import stretch_body.device
from stretch_factory.device_mgmt import StretchDeviceMgmt


# #####################################################################################################
class FirmwareVersion():
    """
    Manage comparision of firmware versions
    """
    def __init__(self,version_str):
        self.device='NONE'
        self.major=0
        self.minor=0
        self.bugfix=0
        self.protocol=0
        self.valid=False
        self.from_string(version_str)
    def __str__(self):
        return self.to_string()
    def to_string(self):
        """
        Version is represented as Stepper.v0.0.1p0 for example
        """
        return self.device+'.v'+str(self.major)+'.'+str(self.minor)+'.'+str(self.bugfix)+'p'+str(self.protocol)

    def __gt__(self, other):
        if not self.valid or not other.valid:
            return False
        if self.protocol>other.protocol:
            return True
        if self.protocol<other.protocol:
            return False
        if self.major > other.major:
            return True
        if self.minor > other.minor:
            return True
        if self.bugfix > other.bugfix:
            return True
        return False

    def __lt__(self, other):
        if not self.valid or not other.valid:
            return False
        if self.protocol<other.protocol:
            return True
        if self.protocol>other.protocol:
            return False
        if self.major < other.major:
            return True
        if self.minor < other.minor:
            return True
        if self.bugfix < other.bugfix:
            return True
        return False

    def __ne__(self,other):
        return not self.__eq__(other)

    def __eq__(self,other):
        if not other or not self.valid or not other.valid:
            return False
        return self.major == other.major and self.minor == other.minor and self.bugfix == other.bugfix and self.protocol==other.protocol

    def same_device(self,d):
        return d==self.device

    def from_string(self,x):
        #X is of form 'Stepper.v0.0.1p0'
        try:
            xl=x.split('.')
            if len(xl) != 4:
                raise Exception('Invalid version len')
            device=xl[0]
            if not (device=='Stepper' or device=='Wacc' or device=='Pimu'):
                raise Exception('Invalid device name ')
            major=int(xl[1][1:])
            minor=int(xl[2])
            bugfix=int(xl[3][0:xl[3].find('p')])
            protocol=int(xl[3][(xl[3].find('p')+1):])
            self.device=device
            self.major=major
            self.minor=minor
            self.bugfix=bugfix
            self.protocol=protocol
            self.valid=True
        except(ValueError,Exception):
            print('Invalid version format in tag: %s'%x)
# #####################################################################################################
class InstalledFirmware():
    """
    Pull the current installed firmware off the robot uCs
    Build config_info of form:
    {'hello-motor-arm': {'board_info': {'board_version': u'Stepper.Irma.V1',
       'firmware_version': u'Stepper.v0.0.1p1',
       'protocol_version': u'p1'},
      'installed_protocol_valid': True,
      'supported_protocols': ['p0', 'p1']}}
    """
    def __init__(self,use_device):
        """
        use_device has form of:
        {'hello-motor-lift': True, 'hello-motor-arm': True, 'hello-motor-right-wheel': True, 'hello-motor-left-wheel': True, 'hello-pimu': True, 'hello-wacc': True}
        """
        self.use_device=use_device
        self.config_info={'hello-motor-lift': None,'hello-motor-arm':None,'hello-motor-left-wheel':None,'hello-motor-right-wheel':None,'hello-pimu':None,'hello-wacc':None}
        for device in self.config_info.keys():
            if self.use_device[device]:
                if device=='hello-wacc':
                    dd=stretch_body.wacc.Wacc()
                elif device == 'hello-pimu':
                    dd = stretch_body.pimu.Pimu()
                else:
                    dd=stretch_body.stepper.Stepper('/dev/'+device)
                dd.startup()
                if dd.board_info['firmware_version'] is not None: #Was able to pull board info from device
                    self.config_info[device]={}
                    self.config_info[device]['board_info'] = dd.board_info.copy()
                    try:
                        self.config_info[device]['supported_protocols']=dd.supported_protocols.keys()
                    except AttributeError:
                        #Older versions of stretch body used a different represenation
                        self.config_info[device]['supported_protocols']=[dd.valid_firmware_protocol]
                    self.config_info[device]['installed_protocol_valid']=(dd.board_info['protocol_version']in self.config_info[device]['supported_protocols'])
                    self.config_info[device]['version']=FirmwareVersion(self.config_info[device]['board_info']['firmware_version'])
                    dd.stop()
                else:
                    self.config_info[device]=None

    def get_supported_protocols(self,device_name):
        if self.is_device_valid(device_name):
            return self.config_info[device_name]['supported_protocols']
        return None
    def get_version(self,device_name):
        if self.is_device_valid(device_name):
            return self.config_info[device_name]['version']
        return None
    def is_device_valid(self,device_name):
        return self.config_info[device_name] is not None

    def is_protocol_supported(self,device_name,p):
        """
        Provide 'p0', etc
        """
        return self.is_device_valid(device_name) and p in self.config_info[device_name]['supported_protocols']

    def max_protocol_supported(self,device_name):
        x=[int(x[1:]) for x in self.config_info[device_name]['supported_protocols']]
        return 'p'+str(max(x))

    def pretty_print(self):
        click.secho(' Currently Installed Firmware '.center(110,'#'),fg="cyan", bold=True)
        for device in self.config_info:
            if self.use_device[device]:
                click.secho('------------ %s ------------'%device.upper(),fg="white", bold=True)
                if self.config_info[device]:
                    click.echo('Installed Firmware: %s'%self.config_info[device]['board_info']['firmware_version'])
                    x=" , ".join(["{}"]*len(self.config_info[device]['supported_protocols'])).format(*self.config_info[device]['supported_protocols'])
                    click.echo('Installed Stretch Body supports protocols: '+x)
                    if self.config_info[device]['installed_protocol_valid']:
                        click.secho('Installed protocol %s : VALID'%self.config_info[device]['board_info']['protocol_version'])
                    else:
                        click.secho('Installed protocol %s : INVALID'%self.config_info[device]['board_info']['protocol_version'],fg="yellow")
                else:
                    click.secho('Device not found')
# #####################################################################################################
class AvailableFirmware():
    def __init__(self,use_device):
        self.use_device=use_device
        self.repo=None
        self.repo_path=None
        self.versions = {}
        for d in self.use_device:
            if self.use_device[d]:
                self.versions[d]=[] #List of available versions for that device
        self.__clone_firmware_repo()
        self.__get_available_firmware_versions()

    def __clone_firmware_repo(self):
        print('Collecting information...')
        self.repo_path = '/tmp/stretch_firmware_update'
        if not os.path.isdir(self.repo_path):
            #print('Cloning latest version of Stretch Firmware to %s'% self.repo_path)
            git.Repo.clone_from('https://github.com/hello-robot/stretch_firmware',  self.repo_path)

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
        click.secho(' Currently Tagged Versions of Stretch Firmware on Master Branch '.center(110,'#'),fg="cyan", bold=True)
        for device_name in self.versions.keys():
            click.secho('---- %s ----'%device_name.upper(), fg="white", bold=True)
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
                    if (v.device == 'Stepper' and device_name in ['hello-motor-lift','hello-motor-arm','hello-motor-left-wheel','hello-motor-right-wheel']) or\
                        (v.device == 'Wacc' and device_name=='hello-wacc') or\
                        (v.device == 'Pimu' and device_name=='hello-pimu'):
                            self.versions[device_name].append(v)



    def get_most_recent_version(self,device_name,supported_protocols):
        """
        For the device and supported protocol versions (eg, '['p0','p1']'), return the most recent version (type FirmwareVersion)
        """
        if len(self.versions[device_name])==0:
            return None
        recent=None
        s=[int(x[1:]) for x in supported_protocols ]
        supported_versions=[]
        for v in self.versions[device_name]:
            if v.protocol in s:
                supported_versions.append(v)
        for sv in supported_versions:
            if recent is None or sv>recent:
                recent=sv
        return recent


    def get_remote_branches(self):
        branches=[]
        print('Collecting information...')
        for ref in self.repo.git.branch('-r').split('\n'):
            sys.stdout.write('.')
            sys.stdout.flush()
            branches.append(ref)
        print('\n')
        branches=[b.strip(' ') for b in branches if b.find('HEAD')==-1]
        branches.remove('origin/master') #Move master to position 0
        branches=['origin/master']+branches
        return branches

# #####################################################################################################
class RecommendedFirmware():
    def __init__(self,use_device,installed=None,available=None):
        self.use_device=use_device
        self.fw_installed = InstalledFirmware(use_device) if installed is None else installed
        self.fw_available= AvailableFirmware(use_device) if available is None else available
        self.recommended = {}
        self.__get_recommend_updates()

    def __get_recommend_updates(self):
        for device_name in self.use_device.keys():
            if self.use_device[device_name]:
                if self.fw_installed.is_device_valid(device_name): #Len 0 if device not found
                    self.recommended[device_name]=self.fw_available.get_most_recent_version(device_name, self.fw_installed.get_supported_protocols(device_name))
                else:
                    self.recommended[device_name]=None

    def pretty_print(self):
        click.secho(' Recommended Firmware Updates '.center(110,'#'), fg="cyan",bold=True)
        print('\n')
        click.secho('%s | %s | %s | %s ' % ('DEVICE'.ljust(25), 'INSTALLED'.ljust(25), 'RECOMMENDED'.ljust(25), 'ACTION'.ljust(25)), fg="cyan", bold=True)
        click.secho('-'*110,fg="cyan", bold=True)
        for device_name in self.recommended.keys():
            dev_out=device_name.upper().ljust(25)
            installed_out=''.ljust(25)
            rec_out = ''.ljust(25)
            action_out = ''.ljust(25)
            if not self.fw_installed.is_device_valid(device_name):
                installed_out='No device available'.ljust(25)
            else:
                version = self.fw_installed.get_version(device_name)
                installed_out=str(version).ljust(25)
                if self.recommended[device_name]==None:
                   rec_out='None (might be on dev branch)'.ljust(25)
                else:
                    rec_out=str(self.recommended[device_name]).ljust(25)
                    if self.recommended[device_name] > version:
                        action_out='Upgrade recommended'.ljust(25)
                    elif self.recommended[device_name] < version:
                        action_out='Downgrade recommended'.ljust(25)
                    else:
                        action_out = 'At most recent version'.ljust(25)
            print('%s | %s | %s | %s ' %(dev_out,installed_out,rec_out,action_out))
# #####################################################################################################

log_device=stretch_body.device.Device(req_params=False)

def user_msg_log(msg,user_display=True,fg=None,bg=None,bold=False):
    if user_display:
        click.secho(str(msg),fg=fg, bg=bg,bold=bold)
    log_device.logger.debug(str(msg))

class FirmwareUpdater():
    def __init__(self,use_device):
        self.use_device=use_device
        self.fw_installed = InstalledFirmware(use_device)
        for device_name in self.use_device.keys():
            self.use_device[device_name]=self.use_device[device_name] and self.fw_installed.is_device_valid(device_name)
        self.fw_available= AvailableFirmware(use_device)
        self.fw_recommended=RecommendedFirmware(use_device,self.fw_installed,self.fw_available)
        self.target=self.fw_recommended.recommended.copy()

    def startup(self):
        #if not self.__check_ubuntu_version():
        #    print('Firmware Updater does not work on Ubuntu 20.04 currently. Please try again in Ubuntu 18.04')
         #   return False
        if self.__check_arduino_cli_install():
            self.__create_arduino_config_file()
            return True
        return False

    def __create_arduino_config_file(self):
        arduino_config = {'board_manager': {'additional_urls': []},
                          'daemon': {'port': '50051'},
                          'directories': {'data': os.environ['HOME'] + '/.arduino15',
                                          'downloads': os.environ['HOME'] + '/.arduino15/staging',
                                          'user': self.fw_available.repo_path + '/arduino'},
                          'library': {'enable_unsafe_install': False},
                          'logging': {'file': '', 'format': 'text', 'level': 'info'},
                          'metrics': {'addr': ':9090', 'enabled': True},
                          'sketch': {'always_export_binaries': False},
                          'telemetry': {'addr': ':9090', 'enabled': True}}
        with open(self.fw_available.repo_path + '/arduino-cli.yaml', 'w') as yaml_file:
            yaml.dump(arduino_config, yaml_file, default_flow_style=False)

    def __check_ubuntu_version(self):
        res = Popen('cat /etc/lsb-release | grep DISTRIB_RELEASE', shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read().strip(b'\n')
        return res==b'DISTRIB_RELEASE=18.04'
        
    def __check_arduino_cli_install(self):
        target_version=b'0.24.0'#0.18.3'
        version='None'
        res=Popen('arduino-cli version', shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()
        do_install=False
        if not(res[:11]==b'arduino-cli'):
            do_install=True
        else:
            version=res[res.find(b'Version:')+9:res.find(b' Commit')]
            if version!=target_version:
                do_install=True
        if do_install:
            click.secho('WARNING:---------------------------------------------------------------------------------',fg="yellow", bold=True)
            click.secho('WARNING: Compatible version of arduino_cli not installed. ',fg="yellow", bold=True)
            click.secho('Requires version %s. Installed version of %s'%(target_version,version))
            if click.confirm('Install now?'):
                os.system('curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin/ sh -s %s'%target_version.decode('utf-8'))
                os.system('arduino-cli config init')
                os.system('arduino-cli core install arduino:samd@1.6.21')
                return True
            else:
                return False
        return True

    def pretty_print_target(self):
        click.secho(' UPDATING FIRMWARE TO... '.center(110,'#'), fg="cyan", bold=True)
        for device_name in self.target.keys():
            if self.use_device[device_name]:
                if not self.fw_installed.is_device_valid(device_name):
                    print('%s | No target available' % device_name.upper().ljust(25))
                else:
                    v_curr=self.fw_installed.get_version(device_name)
                    v_targ=self.target[device_name]
                    if v_targ is None:
                        rec = 'No target available'
                    elif v_curr > v_targ:
                        rec = 'Downgrading to %s' % self.target[device_name]
                    elif v_curr<v_targ:
                        rec = 'Upgrading to %s' % self.target[device_name]
                    else:
                        rec = 'Reinstalling %s' % self.target[device_name]
                    print('%s | %s ' % (device_name.upper().ljust(25), rec.ljust(40)))

    def print_upload_warning(self):
        click.secho('------------------------------------------------', fg="yellow", bold=True)
        click.secho('WARNING: (1) Updating robot firmware should only be done by experienced users', fg="yellow", bold=True)
        click.secho('WARNING: (2) Do not have other robot processes running during update', fg="yellow", bold=True)
        click.secho('WARNING: (3) Leave robot powered on during update', fg="yellow", bold=True)
        if self.use_device['hello-motor-lift']:
            click.secho('WARNING: (4) Ensure Lift has support clamp in place', fg="yellow", bold=True)
            click.secho('WARNING: (5) Lift may make a loud noise during programming. This is normal.', fg="yellow", bold=True)
        click.secho('------------------------------------------------', fg="yellow", bold=True)


    def do_update(self,no_prompts=False,repo_path=None,verbose=False):
        # Return True if system was upgraded
        # Return False if system was not upgraded / error happened
        self.num_update=0

        #Count how many updates doing
        for device_name in self.target.keys():
            if self.fw_installed.is_device_valid(device_name) and self.target[device_name] is not None:
                self.num_update=self.num_update+1

        self.pretty_print_target()

        if not self.num_update:
            click.secho('System is up to date. No updates to be done', fg="yellow",bold=True)
            return False
        self.print_upload_warning()
        self.fw_updated={}
        if no_prompts or click.confirm('Proceed with update??'):
            for device_name in self.target.keys():
                self.fw_updated[device_name]=False
                if self.target[device_name] is not None:
                    self.fw_updated[device_name]=self.flash_firmware_update(device_name,self.target[device_name].to_string(),repo_path=repo_path,verbose=verbose)
                    if not self.fw_updated[device_name]:
                        return False
            click.secho('---- Firmware Update Complete!', fg="cyan",bold=True)
            success=self.post_firmware_update()
            return success
        return True

    def do_update_to(self,verbose=False, no_prompts=False):
        # Return True if system was upgraded
        # Return False if system was not upgraded / error happened
        click.secho(' Select target firmware versions '.center(60,'#'), fg="cyan", bold=True)
        for device_name in self.fw_recommended.recommended.keys():
            if self.use_device[device_name]:
                vs=self.fw_available.versions[device_name]
                if len(vs) and self.fw_recommended.recommended[device_name] is not None:
                    print('')
                    click.secho('---------- %s [%s]-----------'%(device_name.upper(),str(self.fw_installed.get_version(device_name))), fg="blue", bold=True)
                    default_id=0
                    for i in range(len(vs)):
                        if vs[i]==self.fw_recommended.recommended[device_name]:
                            default_id=i
                        print('%d: %s'%(i,vs[i]))
                    print('----------------------')
                    vt=None
                    while vt==None:
                        id = click.prompt('Please enter desired version id [Recommended]', default=default_id)
                        if id>=0 and id<len(vs):
                            vt=vs[id]
                        else:
                            click.secho('Invalid ID', fg="red" )
                    print('Selected version %s for device %s'%(vt,device_name))
                    self.target[device_name]=vt
        print('')
        print('')
        return self.do_update(verbose=verbose, no_prompts=no_prompts)

    def do_update_to_path(self,path_name,verbose=False, no_prompts=False):
        # Burn the Head of the branch to each board regardless of what is currently installed
        click.secho('>>> Flashing firmware from path %s ' % path_name, fg="cyan", bold=True)
        # Check that version of target path is compatible
        for device_name in self.target.keys():
            if self.use_device[device_name]:
                sketch_name = self.get_sketch_name(device_name)
                target_version = self.get_firmware_version_from_path(sketch_name, path_name)
                if target_version is None:
                    return False
                self.target[device_name] = target_version
                path_protocol = 'p' + str(target_version.protocol)
                if not self.fw_installed.is_protocol_supported(device_name, path_protocol):
                    click.secho('---------------------------', fg="yellow")
                    click.secho('Target firmware path of %s is incompatible with installed Stretch Body for device %s' % (path_name, device_name), fg="yellow")
                    x = " , ".join(["{}"] * len(self.fw_installed.get_supported_protocols(device_name))).format(*self.fw_installed.get_supported_protocols(device_name))
                    click.secho('Installed Stretch Body supports protocols %s' % x, fg="yellow")
                    click.secho('Target path supports protocol %s' % path_protocol, fg="yellow")
                    if path_protocol > self.fw_installed.max_protocol_supported(device_name):
                        click.secho('Upgrade Stretch Body first...', fg="yellow")
                    else:
                        click.secho('Downgrade Stretch Body first...', fg="yellow")
                    return False
        repo_path=path_name[:path_name.rfind('arduino')]
        return self.do_update(repo_path=repo_path,verbose=verbose,no_prompts=no_prompts)

    def do_update_to_branch(self,verbose=False, no_prompts=False):
        # Return True if system was upgraded
        # Return False if system was not upgraded / error happened
        click.secho(' Select target branch '.center(60,'#'), fg="cyan", bold=True)
        branches=self.fw_available.get_remote_branches()
        for id in range(len(branches)):
            print('%d: %s' % (id, branches[id]))
        print('')
        branch_name=None
        while branch_name == None:
            try:
                id = click.prompt('Please enter desired branch id',default=0)
            except click.exceptions.Abort:
                return False
            if id >= 0 and id < len(branches):
                branch_name=branches[id]
            else:
                click.secho('Invalid ID', fg="red")
        print('Selected branch %s'%branch_name )
        #Check that version of target branch is compatible
        for device_name in self.target.keys():
            if self.use_device[device_name]:
                sketch_name=self.get_sketch_name(device_name)
                target_version=self.get_firmware_version_from_git(sketch_name, branch_name)
                self.target[device_name]=target_version
                git_protocol = 'p'+str(target_version.protocol)
                if not self.fw_installed.is_protocol_supported(device_name,git_protocol):
                    click.secho('---------------------------', fg="yellow")
                    click.secho('Target firmware branch of %s is incompatible with installed Stretch Body for device %s'%(branch_name,device_name),fg="yellow")
                    x = " , ".join(["{}"] * len(self.fw_installed.get_supported_protocols(device_name))).format(*self.fw_installed.get_supported_protocols(device_name))
                    click.secho('Installed Stretch Body supports protocols %s'%x,fg="yellow")
                    click.secho('Target branch supports protocol %s'%git_protocol,fg="yellow")
                    if git_protocol>self.fw_installed.max_protocol_supported(device_name):
                        click.secho('Upgrade Stretch Body first...',fg="yellow")
                    else:
                        click.secho('Downgrade Stretch Body first...',fg="yellow")
                    return False
        return self.do_update(verbose=verbose,no_prompts=no_prompts)


    def flash_stepper_calibration(self,device_name):
        if device_name == 'hello-motor-arm' or device_name == 'hello-motor-lift' or device_name == 'hello-motor-right-wheel' or device_name == 'hello-motor-left-wheel':
                click.secho(' Flashing Stepper Calibration: %s '.center(110,'#') % device_name, fg="cyan",bold=True)
                time.sleep(1.0)
                motor = stretch_body.stepper.Stepper('/dev/' + device_name)
                motor.startup()
                if not motor.hw_valid:
                    click.secho('Failed to startup stepper %s' % device_name, fg="red", bold=True)
                else:
                    print('Writing gains to flash...')
                    motor.write_gains_to_flash()
                    motor.push_command()
                    print('Gains written to flash')
                    print('')
                    print('Reading calibration data from YAML...')
                    data = motor.read_encoder_calibration_from_YAML()
                    print('Writing calibration data to flash...')
                    motor.write_encoder_calibration_to_flash(data)
                    print('Successful write of FLASH.')
                    self.wait_on_device(device_name)
                    motor.board_reset()
                    motor.push_command()
                    motor.transport.ser.close()
                    time.sleep(2.0)
                    self.wait_on_device(device_name)
                    print('Successful return of device to bus.')




    def post_firmware_update(self):
        #Return True if no errors
        for device_name in self.target.keys():
            if self.fw_updated[device_name]:
                self.flash_stepper_calibration(device_name)
                time.sleep(2.0)  # Give time to get back on bus
                s = StretchDeviceMgmt([device_name])
                s.reset_all()
                if not self.wait_on_device(device_name):
                    print('Failed to return to bus')
                    return False
        print('')
        click.secho(' Confirming Firmware Updates '.center(110,'#'), fg="cyan", bold=True)
        self.fw_installed = InstalledFirmware(self.use_device) #Pull the currently installed system from fw
        n_failure=0
        for device_name in self.target.keys():
            if self.use_device[device_name]:
                if not self.fw_installed.is_device_valid(device_name): #Device may not have come back on bus
                    print('%s | No device available' % device_name.upper().ljust(25))
                    n_failure=n_failure+1
                else:
                    v_curr =self.fw_installed.get_version(device_name)  # Version that is now on the board
                    if v_curr ==  self.target[device_name]:
                        click.secho('%s | %s ' % (device_name.upper().ljust(25), 'Installed firmware matches target'.ljust(40)),fg="green")
                    else:
                        click.secho('%s | %s ' % (device_name.upper().ljust(25), 'Firmware update failure!!'.ljust(40)),fg="red", bold=True)
                        n_failure=n_failure+1
        if n_failure !=0:
            click.secho('#'*110,fg="red", bold=True)
            click.secho('Firmware update reported %d failures.\nTo remedy failures power down and the power up the robot and try again.'%n_failure,fg="red", bold=True)
            return False
        return True

    def get_firmware_version_from_path(self,sketch_name,path):
        file_path = path+'/'+sketch_name+'/Common.h'
        try:
            f=open(file_path,'r')
        except IOError:
            click.secho('Invalid path provided. Path should should have sketch directories under it',fg="red", bold=True)
            return None
        lines=f.readlines()
        for l in lines:
            if l.find('FIRMWARE_VERSION')>=0:
                version=l[l.find('"')+1:-2] #Format of: '#define FIRMWARE_VERSION "Wacc.v0.0.1p1"\n'
                return FirmwareVersion(version)
        return None

    def get_firmware_version_from_git(self,sketch_name,tag):
        #click.secho('---------------Git Checkout-------------------------', fg="green")
        os.chdir(self.fw_available.repo_path)
        os.system('git checkout ' + tag +' >/dev/null 2>&1')
        #print('Checked out out firmware %s from Git for %s' % (tag,sketch_name))
        file_path = self.fw_available.repo_path+'/arduino/'+sketch_name+'/Common.h'
        f=open(file_path,'r')
        lines=f.readlines()
        for l in lines:
            if l.find('FIRMWARE_VERSION')>=0:
                version=l[l.find('"')+1:-2] #Format of: '#define FIRMWARE_VERSION "Wacc.v0.0.1p1"\n'
                return FirmwareVersion(version)
        return None

    def get_sketch_name(self,device_name):
        if device_name=='hello-motor-left-wheel' or device_name=='hello-motor-right-wheel' or device_name=='hello-motor-arm' or device_name=='hello-motor-lift':
            return 'hello_stepper'
        if device_name == 'hello-wacc':
            return 'hello_wacc'
        if device_name == 'hello-pimu':
            return 'hello_pimu'

    def exec_process(self,cmdline, silent, input=None, **kwargs):
        """Execute a subprocess and returns the returncode, stdout buffer and stderr buffer.
           Optionally prints stdout and stderr while running."""
        try:
            sub = Popen(cmdline, stdin=PIPE, stdout=PIPE, stderr=PIPE,
                                   **kwargs)
            stdout, stderr = sub.communicate(input=input)
            returncode = sub.returncode
            if not silent:
                sys.stdout.write(stdout.decode('utf-8'))
                sys.stderr.write(stderr.decode('utf-8'))
        except OSError as e:
            if e.errno == 2:
                raise RuntimeError('"%s" is not present on this system' % cmdline[0])
            else:
                raise
        if returncode != 0:
            raise RuntimeError('Got return value %d while executing "%s", stderr output was:\n%s' % (
            returncode, " ".join(cmdline), stderr.rstrip(b"\n")))
        return stdout

    # ###################################
    def is_device_present(self,device_name):
        try:
            self.exec_process(['ls', '/dev/'+device_name], True)
            return True
        except RuntimeError as e:
            return False

    def wait_on_device(self,device_name,timeout=10.0):
        #Wait for device to appear on bus for timeout seconds
        print('Waiting for device %s to return to bus.'%device_name)
        ts=time.time()
        itr=0
        while(time.time()-ts<timeout):
            if self.is_device_present(device_name):
                return True
            itr=itr+1
            if itr % 5 == 0:
                sys.stdout.write('.')
                sys.stdout.flush()
            time.sleep(0.1)
        return False

    def get_port_name(self, device_name):
        try:
            port_name = Popen("ls -l /dev/" + device_name, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read().strip().split()[-1]
            if not type(port_name)==str:
                port_name=port_name.decode('utf-8')
            return port_name
        except IndexError:
            return None

    def does_stepper_have_encoder_calibration_YAML(self,device_name):
        dd = stretch_body.stepper.Stepper('/dev/' + device_name)
        return len(dd.read_encoder_calibration_from_YAML())!=0

    def flash_firmware_update(self,device_name, tag,repo_path=None,verbose=False):
        click.secho('-------- FIRMWARE FLASH %s | %s ------------'%(device_name,tag), fg="cyan", bold=True)
        config_file = self.fw_available.repo_path + '/arduino-cli.yaml'

        user_msg_log('Config: '+str(config_file), user_display=verbose)
        user_msg_log('Repo: '+str(repo_path), user_display=verbose)

        sketch_name=None
        if device_name == 'hello-motor-left-wheel' or device_name == 'hello-motor-right-wheel' or device_name == 'hello-motor-arm' or device_name == 'hello-motor-lift':
            sketch_name = 'hello_stepper'
        if device_name == 'hello-wacc':
            sketch_name = 'hello_wacc'
        if device_name == 'hello-pimu':
            sketch_name = 'hello_pimu'

        if sketch_name=='hello_stepper' and not self.does_stepper_have_encoder_calibration_YAML(device_name):
            print('Encoder data has not been stored for %s and may be lost. Aborting firmware flash.'%device_name)
            return False

        s = StretchDeviceMgmt([device_name])
        if not s.reset(device_name):
            return False

        print('Looking for device %s on bus' % device_name)
        if not self.wait_on_device(device_name, timeout=5.0):
            print('Failure: Device not on bus.')
            return False
        port_name = self.get_port_name(device_name)
        if port_name is not None and sketch_name is not None:

            print('Starting programming. This will take about 5s...')
            if repo_path is None:
                os.chdir(self.fw_available.repo_path)
                os.system('git checkout '+tag+'>/dev/null 2>&1')
                src_path=self.fw_available.repo_path
            else:
                src_path=repo_path

            compile_command = 'arduino-cli compile --config-file %s --fqbn hello-robot:samd:%s %s/arduino/%s'%(config_file,sketch_name,src_path,sketch_name)
            user_msg_log(compile_command,user_display=verbose)
            c=Popen(compile_command, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip()
            cc = c.split(b'\n')
            user_msg_log(c, user_display=verbose)

            # In version 0.18.x the last line after compile is: Sketch uses xxx bytes (58%) of program storage space. Maximum is yyy bytes.
            #In version 0.24.x this is now on line 0.
            #Need a more robust way to determine successful compile. Works for now.
            success=str(cc[0]).find('Sketch uses')!=-1
            if not success:
                print('Firmware failed to compile %s at %s' % (sketch_name,src_path))
                return False
            else:
                print('Success in firmware compile')

            upload_command = 'arduino-cli upload  --config-file %s -p /dev/%s --fqbn hello-robot:samd:%s %s/arduino/%s' % (config_file, port_name, sketch_name, src_path,sketch_name)
            user_msg_log(upload_command,user_display=verbose)
            u = Popen(upload_command, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip()
            uu = u.split(b'\n')
            user_msg_log(u, user_display=False)
            if verbose:
                print(upload_command)
                # Pretty print the result
                for l in uu:
                    k = l.split(b'\r')
                    if len(k) == 1:
                        print(k[0].decode('utf-8'))
                    else:
                        for m in k:
                            print(m.decode('utf-8'))

            success = uu[-1] == b'CPU reset.'
            if not success:
                print('Firmware flash. Failed to upload to %s' % (port_name))
            else:
                print('Success in firmware flash.')
                time.sleep(2.0) #Give time to get back on bus
                s = StretchDeviceMgmt([device_name])
                s.reset_all()
                if self.wait_on_device(device_name):
                    return True
            print('Failure for device %s to return to USB bus after upload'%device_name)
            return False
        else:
            print('Firmware update %s. Failed to find device %s'%(tag,device_name))
            return False
