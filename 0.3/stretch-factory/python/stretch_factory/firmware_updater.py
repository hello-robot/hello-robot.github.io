#!/usr/bin/env python

import click
import os
from subprocess import Popen, PIPE, call, DEVNULL
import stretch_body.stepper
import stretch_body.pimu
import stretch_body.wacc
import stretch_body.device
import yaml
import time
import sys
import stretch_body.device

import stretch_body.hello_utils
import shlex
from stretch_factory.firmware_available import FirmwareAvailable
from stretch_factory.firmware_recommended import FirmwareRecommended
from stretch_factory.firmware_installed import FirmwareInstalled
from stretch_factory.firmware_version import FirmwareVersion
import stretch_factory.firmware_utils as fwu

import stretch_factory.hello_device_utils as hdu



class FirmwareUpdater():
    def __init__(self, use_device,args):
        #use_device = {'hello-motor-arm': True, 'hello-motor-right-wheel': True, 'hello-motor-left-wheel': True, 'hello-pimu': True, 'hello-wacc': True,'hello-motor-lift': True}
        self.ready_to_run = False
        self.resume_tmp_filename='/tmp/REx_firmware_updater_resume.yaml'
        self.args=args
        state_from_yaml = self.from_yaml()
        self.home_dir = os.path.expanduser('~')
        self.stepper_type = None

        if args.resume:
            if not state_from_yaml:
                click.secho('WARNING: A previous firmware update is not available. Unable to resume', fg="yellow",bold=True)
                self.ready_to_run = False
                return
            else:

                self.state = state_from_yaml  # Use the disk version
                devs=' '
                for d in self.state['use_device']:
                    if self.state['use_device'][d]:
                        devs=devs+' '+str(d)+' |'
                devs=devs[:-1]
                click.secho('\nResuming firmware updates for:%s '%devs, fg="green",bold=True)
                print('')
                self.ready_to_run = True
        else:
            if state_from_yaml:
                click.secho('WARNING: A previous firmware update is incomplete', fg="yellow", bold=True)
                click.secho('WARNING: Run REx_firmware_updater.py --resume', fg="yellow", bold=True)
                click.secho('WARNING: Or delete file %s and try again.'%self.resume_tmp_filename, fg="yellow",bold=True)
                self.ready_to_run = False
                return
    
            self.state = {}
            self.state['use_device']=use_device
            self.state['verbose'] = args.verbose
            self.state['no_prompts'] = args.no_prompts
            self.state['install_version'] = args.install_version
            self.state['install_path'] = args.install_path
            self.state['install_branch'] = args.install_branch
            if self.state['install_path'] and self.state['install_path'][0] != '/':
                    self.state['install_path'] = os.getcwd() + '/' + self.state['install_path']
            self.state['completed']={}
            for d in use_device:
                if use_device[d]:
                    self.state['completed'][d] = {'flash': False,
                                                            'return_to_bus': False,
                                                            'establish_comms':False,
                                                            'version_validate': False,
                                                            'calibration_flash': False,
                                                            'return_to_bus2': False}


        #Check that all devices targeted can be updated
        self.fw_installed = FirmwareInstalled(self.state['use_device'])
        all_valid=True
        for d in self.state['use_device']:
            if self.state['use_device'][d] and not self.fw_installed.is_device_valid(d):
                click.secho('WARNING: Device %s is not valid. Unable to attempt the firmware update. Skipping device.'%d, fg="yellow", bold=True)
                self.state['use_device'][d]=False
                all_valid=False

        # if not all_valid:
        #     mapping = hdu.get_hello_ttyACMx_mapping()
        #     if len(mapping['missing']):
        #         print('')
        #         click.secho('------------------------------------------', fg="yellow", bold=True)
        #         click.secho('WARNING: Some devices are not on the bus:', fg="yellow", bold=True)
        #         for k in mapping['hello']:
        #             print('%s | %s'%(k,mapping['hello'][k]))
        #         click.secho('------------------------------------------', fg="yellow", bold=True)
        #         click.secho('WARNING: Power cycle the robot and try again', fg="yellow",bold=True)
        #         click.secho('WARNING: Otherwise contact Hello Robot support', fg="yellow", bold=True)
        #         print('')
        #     self.ready_to_run=False
        #     return

        self.fw_available = FirmwareAvailable(self.state['use_device'])
        self.fw_recommended = FirmwareRecommended(self.state['use_device'], self.fw_installed, self.fw_available)

        self.create_arduino_config_file()
        self.ready_to_run = fwu.check_arduino_cli_install(self.state['no_prompts'])

        # Set the target version to flash to recommended for each device
        #This dict has a FirmwareVersion target for each device that has a valid (and desired) update
        self.target = {}

        if args.resume:
            for d in self.state['use_device']:
                if self.state['use_device'][d]:
                    self.target[d] = FirmwareVersion(self.state['target_str'][d])
        else:
            self.state['repo_path']=None

            #Default target is recommended
            for d in self.state['use_device']:
                if self.state['use_device'][d]:
                    if d in self.fw_recommended.recommended:
                        self.target[d] = self.fw_recommended.recommended[d]

            #Now overwrite default target if given command line options
            if args.install_version:
                self.ready_to_run = self.ready_to_run and self.set_target_from_install_version()

            if self.state['install_path']:
                self.ready_to_run = self.ready_to_run and self.set_target_from_install_path(self.state['install_path'])

            if args.install_branch:
                self.ready_to_run = self.ready_to_run and self.set_target_from_install_branch()

            # Store updated target for potential future resume
            self.state['target_str'] = {}
            for d in self.target:
                if self.target[d]:
                    self.state['target_str'][d] = self.target[d].to_string()
                else:
                    self.state['target_str'][d] ='None'

        # Count how many updates doing
        num_update = 0
        for device_name in self.target:
            if self.fw_installed.is_device_valid(device_name):
                num_update = num_update + 1
        self.pretty_print_target()
        if not num_update:
            if not args.resume:
                click.secho('No updates to be done', fg="yellow", bold=True)
            else:
                click.secho('WARNING: Unable to resume update. Not all devices returned to bus successfully', fg="red", bold=True)
                click.secho('WARNING: Power cycle robot.', fg="red", bold=True)
                click.secho('WARNING: Then run: REx_firmware_updater.py --resume', fg="red", bold=True)
            self.ready_to_run=False
        #At this point self.state dictionary has all information needed to run an update cycle

    def to_yaml(self):
        with open(self.resume_tmp_filename, 'w') as yaml_file:
         yaml.dump(self.state, yaml_file, default_flow_style=False)

    def from_yaml(self):
        try:
            with open(self.resume_tmp_filename, 'r') as s:
                return  yaml.load(s, Loader=yaml.FullLoader)
        except IOError:
            return None

    def delete_yaml(self):
        if os.path.exists(self.resume_tmp_filename):
            os.system('rm %s'%self.resume_tmp_filename)

# ########################################################################################################3

    def get_port_from_user(self):
        mapping = hdu.get_hello_ttyACMx_mapping()
        click.secho('-------- Available Ports ------------', fg="cyan",bold=True)
        m=[]
        for a in mapping['ACMx']:
            if mapping['ACMx'][a] is None:
                print('%d | %s' % (len(m), a))
                m.append(a)
        dp = None
        while dp == None:
            id = click.prompt('Please enter desired port id', default=0)
            if id >= 0 and id < len(mapping['ACMx']):
                dp = m[id]
            else:
                click.secho('Invalid ID', fg="red")
        return dp

    def get_device_from_user(self):
        click.secho('--------Available Devices ------------' , fg="cyan",bold=True)
        m=['hello-motor-arm','hello-motor-left-wheel' ,'hello-motor-right-wheel' ,'hello-motor-lift','hello-wacc','hello-pimu']
        for i in range(len(m)):
            print('%d | %s ' % (i, m[i]))
        dp = None
        while dp == None:
            id = click.prompt('Please enter desired device id', default=0)
            if id >= 0 and id < len(m):
                dp = m[id]
            else:
                click.secho('Invalid ID', fg="red")
        return dp


    def run(self):
        if not self.ready_to_run:
            click.secho('WARNING: Unable to complete firmware update...', fg="yellow", bold=True)
            return False

        self.print_upload_warning()

        #First check that all calibration present
        for device_name in self.target:
            sketch_name = fwu.get_sketch_name(device_name)
            if sketch_name == 'hello_stepper' and not fwu.does_stepper_have_encoder_calibration_YAML(device_name):
                print('Encoder data has not been stored for %s and should be stored first.' % device_name)
                print('First run REx_stepper_calibration_flash_to_YAML.py '+device_name)
                print('Aborting firmware flash.')
                return False

        #self.pretty_print_state()
        #Advance the state machine
        if self.state['no_prompts'] or click.confirm('Proceed with update??'):
            call('sudo echo', shell=True)
            print('\n\n\n')
            #Flash all devices
            for d in self.target:
                click.secho(' %s  '.center(110, '#') % d.upper(), fg="yellow", bold=True)
                click.secho(' %s |  COMPILE AND FLASH FIRMWARE... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if not self.state['completed'][d]['flash']:
                    if self.fw_installed.is_device_valid(d):
                        nretry=3
                        for i in range(nretry):
                            compile_fail, upload_success=self.do_device_flash(d,self.target[d].to_string(),self.state['repo_path'],self.state['verbose'])
                            self.state['completed'][d]['flash'] = not compile_fail and upload_success
                            if self.state['completed'][d]['flash']:
                                break
                            if compile_fail:
                                click.secho('WARNING: Firmware failed to compile. Fix source then try again', fg="red",bold=True)
                                break
                            if not upload_success: #Dont retry if compile failure
                                #It may get here if the usb bus connectoin fails during flash
                                #Attempt to reset the device and then try again

                                click.secho('WARNING: Failed firmware flash for %s'%d, fg='red', bold=True)
                                break
                                # print('Retrying firmware flash for %s'%d)
                                # port=fwu.get_port_name(d)
                                # if port is not None:
                                #     hdu.place_arduino_in_bootloader('/dev/'+port)
                           
                    else:
                        click.secho('WARNING: Unable to flash %s as device not valid'%d, fg="yellow", bold=True)
                        self.state['completed'][d]['flash'] =False

                    if not self.state['completed'][d]['flash']:
                        click.secho('WARNING: Device %s did not flash firmware successfully'%d, fg="red", bold=True)
                        click.secho('WARNING: Power cycle robot.', fg="red", bold=True)
                        click.secho('WARNING: Then run: REx_firmware_updater.py --resume', fg="red", bold=True)
                        self.to_yaml()
                        return False

                click.secho(' %s |   CHECK #1 IF DEVICE RETURNS TO BUS... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if  not self.state['completed'][d]['return_to_bus']:
                    self.state['completed'][d]['return_to_bus']=self.wait_on_return_to_bus(d)

                    if not self.state['completed'][d]['return_to_bus']:
                        click.secho('WARNING: Device %s did not return to bus successfully'%d, fg="red", bold=True)
                        click.secho('WARNING: Power cycle robot.', fg="red", bold=True)
                        click.secho('WARNING: Then run: REx_firmware_updater.py --resume', fg="red", bold=True)
                        self.to_yaml()
                        return False

                time.sleep(3.0) #Give a chance for devices to become ready for comms

                click.secho(' %s |   CHECK IF ESTABLISH COMMS... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if not self.state['completed'][d]['establish_comms']:
                    self.state['completed'][d]['establish_comms'] = self.verify_establish_comms(d)

                    if not self.state['completed'][d]['establish_comms']:
                        click.secho('WARNING: Device %s did not establish comms successfully'%d, fg="red", bold=True)
                        click.secho('WARNING: Power cycle robot.', fg="red", bold=True)
                        click.secho('WARNING: Then run: REx_firmware_updater.py --resume', fg="red", bold=True)
                        self.to_yaml()
                        return False

                click.secho('%s |  CHECK FOR CORRECT VERSION UPDATE... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if not self.state['completed'][d]['version_validate']:
                    self.state['completed'][d]['version_validate'] = self.verify_firmware_version(d)
                    if not self.state['completed'][d]['version_validate']: #If failed, force to try upload again
                        self.state['completed'][d]['flash']=False
                        self.state['completed'][d]['return_to_bus']=False
                        self.state['completed'][d]['establish_comms'] = False
                        click.secho('WARNING: Device %s has not updated to target firmware version'%d, fg="red", bold=True)
                        click.secho('WARNING: Power cycle robot.', fg="red", bold=True)
                        click.secho('WARNING: Then run: REx_firmware_updater.py --resume', fg="red", bold=True)
                        self.to_yaml()
                        return False

                click.secho('%s |  RESTORING CALIBRATION DATA... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if not self.state['completed'][d]['calibration_flash']:
                    self.state['completed'][d]['calibration_flash'] = self.flash_stepper_calibration(d)

                if not self.state['completed'][d]['calibration_flash']:
                    click.secho('WARNING: Device %s failed on encoder calibration flash'%d, fg="red", bold=True)
                    click.secho('WARNING: Power cycle robot.', fg="red", bold=True)
                    click.secho('WARNING: Then run: REx_firmware_updater.py --resume', fg="red", bold=True)
                    self.to_yaml()
                    return False

                click.secho('%s |  CHECK #2 IF RETURNED TO BUS... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if  not self.state['completed'][d]['return_to_bus2']:
                    self.state['completed'][d]['return_to_bus2']=self.wait_on_return_to_bus(d)

                if not self.state['completed'][d]['return_to_bus2']:
                    click.secho('WARNING: Device %s did not return to bus successfully'%d, fg="red", bold=True)
                    click.secho('WARNING: Power cycle robot.', fg="red", bold=True)
                    click.secho('WARNING: Then run: REx_firmware_updater.py --resume', fg="red", bold=True)
                    self.to_yaml()
                    return False
                print('\n\n\n')

            print('')
            click.secho(' CONGRATULATIONS... '.center(110, '#'), fg="cyan", bold=True)
            for d in self.target:
                click.secho('%s | No issues encountered. Firmware updated to %s.'%(d.upper().ljust(25),str(self.target[d])), fg="green", bold=True)
            self.delete_yaml()
            return True

    # ########################################################################################################3

    def all_completed(self,state_name):
        all_completed=True
        for d in self.target:
            all_completed=all_completed and self.state['completed'][d][state_name]
        return all_completed
    
    def extract_stepper_type(self, device_name):
                if 'hello-motor' in device_name:
                    st = stretch_body.stepper.Stepper('/dev/' + device_name)
                    for i in st.supported_protocols.keys():
                        recent_protocol = i.strip('p')
                    if int(recent_protocol) >= 5:
                        if not st.startup():
                            click.secho('FAIL: Unable to establish comms with device %s' % device_name.upper(), fg="red")
                            return False
                        else:
                            if int(st.board_info['protocol_version'].strip('p')) >= 5:
                                self.stepper_type = st.board_info['stepper_type']
                                time.sleep(0.5)
                            st.stop()
                            del st

# ########################################################################################################################
    def do_device_flash(self, device_name, tag, repo_path=None, verbose=False, port_name=None):
        """
        Return compile_fail, upload_success
        """
        config_file = self.fw_available.repo_path + '/arduino-cli.yaml'

        fwu.user_msg_log('Config: ' + str(config_file), user_display=verbose)
        fwu.user_msg_log('Repo: ' + str(repo_path), user_display=verbose)

        sketch_name=fwu.get_sketch_name(device_name)

        if port_name is None:
            print('Looking for device %s on bus' % device_name)
            if not fwu.wait_on_device(device_name, timeout=5.0):
                print('Failure: Device not on bus.')
                return False, False
            port_name = fwu.get_port_name(device_name)

        
        self.extract_stepper_type(device_name)
        fwu.user_msg_log('Device: %s Port: %s' % (device_name, port_name), user_display=verbose)

        if port_name is not None and sketch_name is not None:
            print('Starting programming. This will take about 5s...')
            if repo_path is None:
                os.chdir(self.fw_available.repo_path)
                cmd='git checkout ' + tag + '>/dev/null 2>&1'
                os.system(cmd)
                if verbose:
                    print('Ran: %s from %s'%(cmd,self.fw_available.repo_path))
                src_path = self.fw_available.repo_path
            else:
                src_path = repo_path

            compile_command = 'arduino-cli compile --config-file %s --fqbn hello-robot:samd:%s %s/arduino/%s --export-binaries' % (
            config_file, sketch_name, src_path, sketch_name)
            fwu.user_msg_log(compile_command, user_display=verbose)
            c = Popen(shlex.split(compile_command), shell=False, bufsize=64, stdin=PIPE, stdout=PIPE,
                      close_fds=True).stdout.read().strip()
            if type(c) == bytes:
                c = c.decode("utf-8")
            cc = c.split('\n')
            fwu.user_msg_log(c, user_display=verbose)

            # In version 0.18.x the last line after compile is: Sketch uses xxx bytes (58%) of program storage space. Maximum is yyy bytes.
            # In version 0.24.x +this is now on line 0.
            # Need a more robust way to determine successful compile. Works for now.
            success = (str(cc[0]).find('Sketch uses') != -1)
            if not success:
                print('Firmware failed to compile %s at %s' % (sketch_name, src_path))
                return True, False
            else:
                print('Success in firmware compile')

        #     upload_command = 'arduino-cli upload  --config-file %s -p /dev/%s --fqbn hello-robot:samd:%s %s/arduino/%s' % (
        #     config_file, port_name, sketch_name, src_path, sketch_name)

        #     fwu.user_msg_log(upload_command, user_display=verbose)
        #     u = Popen(shlex.split(upload_command), shell=False, bufsize=64, stdin=PIPE, stdout=PIPE,
        #               close_fds=True).stdout.read().strip()

        #     if type(u) == bytes:
        #         u = u.decode('utf-8')
        #     uu = u.split('\n')
        #     fwu.user_msg_log(u, user_display=False)
        #     if verbose:
        #         print(upload_command)
        #         # Pretty print the result
        #         for l in uu:
        #             k = l.split('\r')
        #             if len(k) == 1:
        #                 print(k[0])
        #             else:
        #                 for m in k:
        #                     print(m)
        #     success = uu[-1] == 'CPU reset.'

        #     if not success:
        #         print('Firmware flash. Failed to upload to %s' % (port_name))
        #         return False, False
        #     else:
        #         click.secho('Success in firmware flash' , fg="green")
        #         return False, True
        # else:
        #     print('Firmware update %s. Failed to find device %s' % (tag, device_name))
        #     return False, False

        ############## bug fix experimental code ################################################
        if device_name == 'hello-pimu':
                fw_file = '/arduino/hello_pimu/build/hello-robot.samd.hello_pimu/hello_pimu.ino.bin'

        if 'hello-motor' in device_name:
            fw_file = '/arduino/hello_stepper/build/hello-robot.samd.hello_stepper/hello_stepper.ino.bin'

        if device_name == 'hello-wacc':
            fw_file = '/arduino/hello_wacc/build/hello-robot.samd.hello_wacc/hello_wacc.ino.bin'

        flash_command = self.home_dir+'/.arduino15/packages/arduino/tools/bossac/1.7.0/bossac -i -d --port='+port_name+ ' -U true -i -e -w -v '+src_path + fw_file+' -R' 
        flash_sts = False, False
        while True:
            try:
                print(f"#### Trying To place {device_name} in bootloader mode #######")
                test_port = hdu.serial.Serial('/dev/'+port_name, baudrate=1200)
                test_port.__del__()
                time.sleep(2)
        
                if hdu.extract_udevadm_info('/dev/'+port_name,'ID_MODEL') == 'Arduino_Zero':
                    click.secho(f'Success {device_name} in bootloader mode, Now Flashing!', fg="green", bold=True)
                    time.sleep(1)
            
                    result = call(flash_command, shell=True, stdout=DEVNULL)
                    if result == 0:
                        click.secho(f'Success Flashing {device_name}', fg="green", bold=True)
                        time.sleep(1)
                        flash_sts = False, True
                        break
                    else:
                        click.secho(f'Flashing {device_name} FAILED', fg="red", bold=True)
                        break
                else:
                    click.secho(f'{device_name} not in bootloader mode, retrying!', fg="yellow", bold=True)
                    time.sleep(1)
                    test_port = hdu.serial.Serial('/dev/'+port_name, baudrate=2000000)
                    time.sleep(1)
                    test_port.__del__()
                    time.sleep(1) 

            except TypeError:
                continue
        return flash_sts
        ############## bug fix experimental code ################################################

# ########################################################################################################3
    def wait_on_return_to_bus(self,device_name):
        click.secho('Checking that device %s returned to bus '%device_name)
        print('It may take several minutes to appear on the USB bus.' )
        ts = time.time()
        found = False
        ntry=30
        for i in range(ntry):
            if not fwu.wait_on_device(device_name, timeout=10.0):
                print('Trying again: %d of %d\n' % (i,ntry))
                # Bit of a hack.Sometimes after a firmware flash the device
                # Doesn't fully present on the USB bus with a serial No for Udev to find
                # In does present as an 'Arduino Zero' product. This will attempt to reset it
                # and re-present to the bus
                time.sleep(1.0)
                click.secho(f'Resetting usb of {device_name} please wait a few seconds', fg="yellow", bold = False)
                call('sudo usbreset \"Arduino Zero\"', shell=True, stdout=DEVNULL)
                time.sleep(2.0)
            else:
                found = True
                break
        if not found:
            click.secho('Device %s failed to return to bus after %f seconds.' % (device_name, time.time() - ts),fg="yellow", bold=True)
            return False
        else:
            click.secho('Device %s returned to bus after %f seconds.' % (device_name, time.time() - ts),fg="green", bold=True)
        return True
# ########################################################################################################3
    def verify_firmware_version(self,device_name):
        fw_installed = FirmwareInstalled({device_name: True})  # Pull the currently installed system from fw
        if not fw_installed.is_device_valid(device_name):  # Device may not have come back on bus
            print('%s | No device available' % device_name.upper().ljust(25))
            print('')
            return False
        else:
            #click.secho(' Confirming Firmware Updates '.center(110, '#'), fg="cyan", bold=True)
            v_curr = fw_installed.get_version(device_name)  # Version that is now on the board
            if v_curr == self.target[device_name]:
                click.secho('PASS: %s | Installed %s | Target %s ' % (device_name.upper().ljust(25), v_curr.to_string().ljust(40),self.target[device_name].to_string().ljust(40)), fg="green")
                return True
            else:
                click.secho('FAIL: %s | Installed %s | Target %s ' % (device_name.upper().ljust(25), v_curr.to_string().ljust(40), self.target[device_name].to_string().ljust(40)),fg="red")
        return False

    def verify_establish_comms(self,device_name):
        if device_name == 'hello-wacc':
            dd = stretch_body.wacc.Wacc()
        elif device_name == 'hello-pimu':
            dd = stretch_body.pimu.Pimu()
        else:
            dd = stretch_body.stepper.Stepper('/dev/' + device_name)
        if not dd.startup():
            click.secho('FAIL: Unable to establish comms with device %s' % device_name.upper(), fg="red")
            return False
        else:
            time.sleep(0.5)
            dd.stop()
            del dd
        click.secho('PASS: Established comms with device %s ' % device_name.upper(),fg="green")
        return True
# ########################################################################################################3
    def flash_stepper_calibration(self, device_name):
        if device_name == 'hello-motor-arm' or device_name == 'hello-motor-lift' or device_name == 'hello-motor-right-wheel' or device_name == 'hello-motor-left-wheel':
            #click.secho(' Flashing Stepper Calibration: %s '.center(70, '#') % device_name, fg="cyan", bold=True)
            if not fwu.wait_on_device(device_name):
                click.secho('Device %s failed to return to bus.' % device_name, fg="red", bold=True)
                return False
            #time.sleep(1.0)
            motor = stretch_body.stepper.Stepper('/dev/' + device_name)
            motor.startup()
            if not motor.hw_valid:
                click.secho('Failed to startup stepper %s' % device_name, fg="red", bold=True)
                return False
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
                print('\n')

                if int(motor.board_info['protocol_version'].strip('p')) >= 5 and self.stepper_type is not None:
                    print('Writing stepper type to flash...')
                    motor.write_stepper_type_to_flash(self.stepper_type)
                    print('Success writing stepper type to Flash')
                    print('\n')

                print('Successful write of FLASH.')
                fwu.wait_on_device(device_name)
                motor.board_reset()
                motor.push_command()
                motor.transport.ser.close()
                time.sleep(2.0) #Give time to return to bus
                return True
        click.secho('Successful flash of device calibration',fg="green")
        return True
# ########################################################################################################3

    def set_target_from_install_version(self):
        # Return True if system was upgraded
        # Return False if systvt = None
        #                     while vt == None:
        #                         id = click.prompt('Please enter desired version id [Recommended]', default=default_id)
        #                         if id >= 0 and id < len(vs):
        #                             vt = vs[id]
        #                         else:
        #                             click.secho('Invalid ID', fg="red")
        #                     print('Selected version %s for device %s' % (vt, device_name))em was not upgraded / error happened
        click.secho(' Select target firmware versions '.center(60, '#'), fg="cyan", bold=True)
        for device_name in self.fw_recommended.recommended.keys():
            if self.state['use_device'][device_name]:
                vs = self.fw_available.versions[device_name]
                if len(vs) and self.fw_recommended.recommended[device_name] is not None:
                    print('')
                    click.secho('---------- %s [%s]-----------' % (
                    device_name.upper(), str(self.fw_installed.get_version(device_name))), fg="blue", bold=True)
                    default_id = 0
                    for i in range(len(vs)):
                        if vs[i] == self.fw_recommended.recommended[device_name]:
                            default_id = i
                        print('%d: %s' % (i, vs[i]))
                    print('----------------------')
                    id = click.prompt('Please enter desired version id [Recommended]', default=default_id)
                    if id >= 0 and id < len(vs):
                        vt = vs[id]
                    else:
                        click.secho('Invalid ID', fg="red")
                print('Selected version %s for device %s' % (vt, device_name))
                self.target[device_name] = vt
        print('')
        print('')
        return True

    def set_target_from_install_path(self, path_name):
        # Burn the Head of the branch to each board regardless of what is currently installed
        click.secho('>>> Flashing firmware from path %s ' % path_name, fg="cyan", bold=True)
        # Check that version of target path is compatible
        for device_name in self.target:
            if self.state['use_device'][device_name]:
                sketch_name = fwu.get_sketch_name(device_name)
                target_version = self.get_firmware_version_from_path(sketch_name, path_name)
                if target_version is None:
                    return False
                self.target[device_name] = target_version
                path_protocol = 'p' + str(target_version.protocol)
                if not self.fw_installed.is_protocol_supported(device_name, path_protocol):
                    click.secho('---------------------------', fg="yellow")
                    click.secho(
                        'Target firmware path of %s is incompatible with installed Stretch Body for device %s' % (
                        path_name, device_name), fg="yellow")
                    x = " , ".join(["{}"] * len(self.fw_installed.get_supported_protocols(device_name))).format(
                        *self.fw_installed.get_supported_protocols(device_name))
                    click.secho('Installed Stretch Body supports protocols %s' % x, fg="yellow")
                    click.secho('Target path supports protocol %s' % path_protocol, fg="yellow")
                    if path_protocol > self.fw_installed.max_protocol_supported(device_name):
                        click.secho('Upgrade Stretch Body first...', fg="yellow")
                    else:
                        click.secho('Downgrade Stretch Body first...', fg="yellow")
                    return False
        self.state['repo_path']=path_name[:path_name.rfind('arduino')]
        return True

    def set_target_from_install_branch(self,):
        # Return True if system was upgraded
        # Return False if system was not upgraded / error happened
        click.secho(' Select target branch '.center(60, '#'), fg="cyan", bold=True)
        branches = self.fw_available.get_remote_branches()
        for id in range(len(branches)):
            print('%d: %s' % (id, branches[id]))
        print('')
        branch_name = None
        while branch_name == None:
            try:
                id = click.prompt('Please enter desired branch id', default=0)
            except click.exceptions.Abort:
                return False
            if id >= 0 and id < len(branches):
                branch_name = branches[id]
            else:
                click.secho('Invalid ID', fg="red")
        print('Selected branch %s' % branch_name)
        self.state['install_branch']=branch_name
        # Check that version of target branch is compatible
        for device_name in self.target:
            if self.state['use_device'][device_name]:
                sketch_name = fwu.get_sketch_name(device_name)
                target_version = self.get_firmware_version_from_git(sketch_name, branch_name)
                self.target[device_name] = target_version
                git_protocol = 'p' + str(target_version.protocol)
                if not self.fw_installed.is_protocol_supported(device_name, git_protocol):
                    click.secho('---------------------------', fg="yellow")
                    click.secho(
                        'Target firmware branch of %s is incompatible with installed Stretch Body for device %s' % (
                        branch_name, device_name), fg="yellow")
                    x = " , ".join(["{}"] * len(self.fw_installed.get_supported_protocols(device_name))).format(
                        *self.fw_installed.get_supported_protocols(device_name))
                    click.secho('Installed Stretch Body supports protocols %s' % x, fg="yellow")
                    click.secho('Target branch supports protocol %s' % git_protocol, fg="yellow")
                    if git_protocol > self.fw_installed.max_protocol_supported(device_name):
                        click.secho('Upgrade Stretch Body first...', fg="yellow")
                    else:
                        click.secho('Downgrade Stretch Body first...', fg="yellow")
                    return False
        return True


# ########################################################################################################################


# ########################################################################################################################
#     def foo(self):
#         all_success = all_success and self.verify_firmware_version(device_name)
#         print('')
#         print('')
#                 # NOTE: Move to exceptions and single device flow, can track where in the flow it fails.
#                 self.post_firmware_update(device_name)
#
#                 if len(no_return):
#                     click.secho('Devices did not return to bus. Power cycle robot', fg="yellow", bold=True)
#                     click.secho('Then run stretch_robot_system_check.py to confirm all devices present', fg="yellow",
#                                 bold=True)
#                     for device_name in no_stepper_return:
#                         click.secho(
#                             'Device %s requires calibration data to be written after power cycle.' % device_name,
#                             fg="yellow", bold=True)
#                         click.secho('After power cycle run: REx_stepper_calibration_YAML_to_flash.py %s' % device_name,
#                                     fg="yellow", bold=True)
#                     return False
#
#                 return all_success
#         return True




    def create_arduino_config_file(self):
        if  self.state['install_path']:
            user_path = self.state['install_path']
        else:
            user_path = self.fw_available.repo_path + '/arduino'
       
        arduino_config = {'board_manager': {'additional_urls': []},
                          'daemon': {'port': '50051'},
                          'directories': {'data': os.environ['HOME'] + '/.arduino15',
                                          'downloads': os.environ['HOME'] + '/.arduino15/staging',
                                          'user': user_path},
                          'library': {'enable_unsafe_install': False},
                          'logging': {'file': '', 'format': 'text', 'level': 'info'},
                          'metrics': {'addr': ':9090', 'enabled': True},
                          'sketch': {'always_export_binaries': False},
                          'telemetry': {'addr': ':9090', 'enabled': True}}
        with open(self.fw_available.repo_path + '/arduino-cli.yaml', 'w') as yaml_file:
            yaml.dump(arduino_config, yaml_file, default_flow_style=False)

    def pretty_print_target(self):
        click.secho(' UPDATING FIRMWARE TO... '.center(110, '#'), fg="cyan", bold=True)
        for device_name in self.target:
            if self.state['use_device'][device_name]:
                if not self.fw_installed.is_device_valid(device_name):
                    print('%s | No target available' % device_name.upper().ljust(25))
                else:
                    v_curr = self.fw_installed.get_version(device_name)
                    v_targ = self.target[device_name]
                    if v_targ is None:
                        rec = 'No target available'
                    elif v_curr > v_targ:
                        rec = 'Downgrading to %s' % self.target[device_name]
                    elif v_curr < v_targ:
                        rec = 'Upgrading to %s' % self.target[device_name]
                    else:
                        rec = 'Reinstalling %s' % self.target[device_name]
                    print('%s | %s ' % (device_name.upper().ljust(25), rec.ljust(40)))
        print('')

    def print_upload_warning(self):
        click.secho('------------------------------------------------', fg="yellow", bold=True)
        click.secho('WARNING: (1) Updating robot firmware should only be done by experienced users', fg="yellow",
                    bold=True)
        click.secho('WARNING: (2) Do not have other robot processes running during update', fg="yellow", bold=True)
        click.secho('WARNING: (3) Leave robot powered on during update', fg="yellow", bold=True)
        if self.state['use_device']['hello-motor-lift']:
            click.secho('WARNING: (4) Ensure Lift has support clamp in place', fg="yellow", bold=True)
            click.secho('WARNING: (5) Lift may make a loud noise during programming. This is normal.', fg="yellow",
                        bold=True)
        click.secho('------------------------------------------------', fg="yellow", bold=True)








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

