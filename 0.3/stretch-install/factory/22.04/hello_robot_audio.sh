#! /bin/bash
sleep 5
pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo
amixer set Master 80%
paplay --device=alsa_output.pci-0000_00_1f.3.analog-stereo /usr/share/sounds/ubuntu/stereo/desktop-login.ogg

