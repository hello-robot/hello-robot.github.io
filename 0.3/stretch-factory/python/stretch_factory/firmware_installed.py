import click
import stretch_body.stepper
import stretch_body.pimu
import stretch_body.wacc
import stretch_body.device
import stretch_body.device
import stretch_body.hello_utils
from stretch_factory.firmware_version import FirmwareVersion

class FirmwareInstalled():
    """
    Pull the current installed firmware off the robot uCs
    Build config_info of form:
    {'hello-motor-arm': {'board_info': {'board_version': u'Stepper.Irma.V1',
       'firmware_version': u'Stepper.v0.0.1p1',
       'protocol_version': u'p1'},
      'installed_protocol_valid': True,
      'supported_protocols': ['p0', 'p1']}}
    """

    def __init__(self, use_device):
        """
        use_device has form of:
        {'hello-motor-lift': True, 'hello-motor-arm': True, 'hello-motor-right-wheel': True, 'hello-motor-left-wheel': True, 'hello-pimu': True, 'hello-wacc': True}

        config_info is a dict like:

        {'hello-motor-lift': {'board_info': {'board_variant': 'Stepper.0',
           'firmware_version': 'Stepper.v0.3.0p2',
           'protocol_version': 'p2',
           'hardware_id': 0},
          'supported_protocols': ['p0', 'p1', 'p2'],
          'installed_protocol_valid': True,
          'version': 'Stepper.v0.3.0p2'},...}
        """
        self.use_device = use_device
        self.config_info = {'hello-motor-lift': None, 'hello-motor-arm': None, 'hello-motor-left-wheel': None,
                            'hello-motor-right-wheel': None, 'hello-pimu': None, 'hello-wacc': None}
        print('Collecting information...')
        for device in self.use_device.keys():
            if self.use_device[device]:
                if device == 'hello-wacc':
                    dd = stretch_body.wacc.Wacc()
                elif device == 'hello-pimu':
                    dd = stretch_body.pimu.Pimu()
                else:
                    dd = stretch_body.stepper.Stepper('/dev/' + device)
                if not dd.startup():
                    click.secho('Unable to communicate with device %s'%device,fg="red", bold=True)
                else:
                    if dd.board_info['firmware_version'] is not None:  # Was able to pull board info from device
                        self.config_info[device] = {}
                        self.config_info[device]['board_info'] = dd.board_info.copy()
                        try:
                            self.config_info[device]['supported_protocols'] = list(dd.supported_protocols.keys())
                        except AttributeError:
                            # Older versions of stretch body used a different represenation
                            self.config_info[device]['supported_protocols'] = [dd.valid_firmware_protocol]
                        self.config_info[device]['installed_protocol_valid'] = (
                                    dd.board_info['protocol_version'] in self.config_info[device]['supported_protocols'])
                        self.config_info[device]['version'] = FirmwareVersion(self.config_info[device]['board_info']['firmware_version'])
                        dd.stop()
                    else:
                        self.config_info[device] = None

    def get_supported_protocols(self, device_name):
        if self.is_device_valid(device_name):
            return self.config_info[device_name]['supported_protocols']
        return None

    def get_version(self, device_name):
        if self.is_device_valid(device_name):
            return self.config_info[device_name]['version']
        return None

    def is_device_valid(self, device_name):
        return self.config_info[device_name] is not None

    def is_protocol_supported(self, device_name, p):
        """
        Provide 'p0', etc
        """
        return self.is_device_valid(device_name) and p in self.config_info[device_name]['supported_protocols']

    def max_protocol_supported(self, device_name):
        x = [int(x[1:]) for x in self.config_info[device_name]['supported_protocols']]
        return 'p' + str(max(x))

    def pretty_print(self):
        click.secho(' Currently Installed Firmware '.center(110, '#'), fg="cyan", bold=True)
        for device in self.config_info:
            if self.use_device[device]:
                click.secho('------------ %s ------------' % device.upper(), fg="white", bold=True)
                if self.config_info[device]:
                    click.echo('Installed Firmware: %s' % self.config_info[device]['board_info']['firmware_version'])
                    x = " , ".join(["{}"] * len(self.config_info[device]['supported_protocols'])).format(
                        *self.config_info[device]['supported_protocols'])
                    click.echo('Installed Stretch Body supports protocols: ' + x)
                    if self.config_info[device]['installed_protocol_valid']:
                        click.secho('Installed protocol %s : VALID' % self.config_info[device]['board_info'][
                            'protocol_version'])
                    else:
                        click.secho('Installed protocol %s : INVALID' % self.config_info[device]['board_info'][
                            'protocol_version'], fg="yellow")
                else:
                    click.secho('Device not found')
