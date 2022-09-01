import unittest
from stretch_factory.firmware_updater import *


class TestFirmwareUpdater(unittest.TestCase):
    def test_all_cycle(self):
        """
        Do N cycles of flashing full robot
        :return:
        """
        use_device = {'hello-motor-lift': True, 'hello-motor-arm': True, 'hello-motor-right-wheel': True,
                      'hello-motor-left-wheel': True, 'hello-pimu': True, 'hello-wacc': True}
        u = FirmwareUpdater(use_device)
        self.assertTrue(u.startup())
        ncycle = 100
        for i in range(ncycle):
            print('################# UPLOAD %d #############3' % i)
            self.assertTrue(u.do_update(no_prompts=True))




