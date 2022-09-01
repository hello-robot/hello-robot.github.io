import unittest
from stretch_factory.device_mgmt import StretchDeviceMgmt



class TestUSBReset(unittest.TestCase):
    def test_reset(self):
        try:
            s = StretchDeviceMgmt()
            s.reset('hello-motor-lift')
            s.reset('hello-motor-arm')
            s.reset('hello-motor-right-wheel')
            s.reset('hello-motor-left-wheel')
            s.reset('hello-pimu')
            s.reset('hello-wacc')
            s.reset('hello-dynamixel-head')
            s.reset('hello-dynamixel-wrist')
            s.reset_all()
        except:
            print('Note: Must be run as sudo python -m unittest <test>')
            self.assertTrue(0)


