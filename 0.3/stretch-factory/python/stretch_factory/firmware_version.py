
class FirmwareVersion():
    """
    Manage comparision of firmware versions
    """

    def __init__(self, version_str):
        self.device = 'NONE'
        self.major = 0
        self.minor = 0
        self.bugfix = 0
        self.protocol = 0
        self.valid = False
        self.from_string(version_str)

    def __str__(self):
        return self.to_string()

    def to_string(self):
        """
        Version is represented as Stepper.v0.0.1p0 for example
        """
        return self.device + '.v' + str(self.major) + '.' + str(self.minor) + '.' + str(self.bugfix) + 'p' + str(
            self.protocol)

    def __gt__(self, other):
        if not self.valid or not other.valid:
            return False
        if self.protocol > other.protocol:
            return True
        if self.protocol < other.protocol:
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
        if self.protocol < other.protocol:
            return True
        if self.protocol > other.protocol:
            return False
        if self.major < other.major:
            return True
        if self.minor < other.minor:
            return True
        if self.bugfix < other.bugfix:
            return True
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __eq__(self, other):
        if not other or not self.valid or not other.valid:
            return False
        return self.major == other.major and self.minor == other.minor and self.bugfix == other.bugfix and self.protocol == other.protocol

    def same_device(self, d):
        return d == self.device

    def from_string(self, x):
        # X is of form 'Stepper.v0.0.1p0'
        try:
            xl = x.split('.')
            if len(xl) != 4:
                raise Exception('Invalid version len')
            device = xl[0]
            if not (device == 'Stepper' or device == 'Wacc' or device == 'Pimu'):
                raise Exception('Invalid device name ')
            major = int(xl[1][1:])
            minor = int(xl[2])
            bugfix = int(xl[3][0:xl[3].find('p')])
            protocol = int(xl[3][(xl[3].find('p') + 1):])
            self.device = device
            self.major = major
            self.minor = minor
            self.bugfix = bugfix
            self.protocol = protocol
            self.valid = True
        except(ValueError, Exception):
            print('Invalid version format in tag: %s' % x)

