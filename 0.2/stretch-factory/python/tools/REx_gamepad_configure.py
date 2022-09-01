#!/usr/bin/env python

import argparse
import usb.core
import inputs
#11c0:5500 /dev/input/by-id/usb-11c0_USB_GAMEPAD-event-joystick (Bottom 2 LED, flashing 4 LED)
#045e:028e /dev/input/by-id/usb-045e_GAME_FOR_WINDOWS-event-joystick (Top 2 LED) (Microsoft Xbox Controller)
#11c1:9101"/dev/input/by-id/usb-11c1_ESM_9103-event-joystick"  (Left 2 LED, Diagonal)

def is_xbox_gamepad_present():
    dev = usb.core.find(idVendor=0x045e, idProduct=0x028e)
    return dev is not None

def configure_gamepad():
    if len(inputs.devices.gamepads)==0:
        print('No gamepads found. Check that USB dongle is installed')
        return

    while inputs.devices.gamepads[0].name == 'ESM 9103':
        print('Gamepad in mode %s. Unplug and plug-in USB dongle now. Hit enter when done.'%inputs.devices.gamepads[0].name)
        raw_input()
        reload(inputs)
        
    if is_xbox_gamepad_present():
        print('Gamepad in configured correctly. Ready to use')
        return


    while not is_xbox_gamepad_present():
        print('Hold down center button for 5s until LEDS change state. Hit enter when done')
        raw_input()
        print('Tap joystick now')
        x=inputs.get_gamepad()
        reload(inputs)
        print('Current gamepad mode selected is %s'%inputs.devices.gamepads[0].name)
    reload(inputs)
    if inputs.devices.gamepads[0].name == 'Microsoft X-Box 360 pad':
        print('Succesfully configured gamepad. Ready to use')

parser=argparse.ArgumentParser(description='Configure the Gamepad controller')
args = parser.parse_args()
configure_gamepad()

