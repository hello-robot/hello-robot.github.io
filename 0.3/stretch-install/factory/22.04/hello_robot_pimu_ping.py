#!/usr/bin/env python3
import stretch_body.pimu
import time
#This script establishes communications with the Pimu at launch then shutsdown
#This has the effect of letting the Pimu know that Ubuntu Desktop is live (which drives its LightBar state machine)

print('Starting up Pimu Ping on %s'%time.asctime())
ping_success=False
for i in range(10):
    p = stretch_body.pimu.Pimu()
    if p.startup():
        print('Successful ping of Pimu on try %d'%i)
        p.stop()
        exit(0)
    p.stop()
    time.sleep(1.0)

print('Failed to ping Pimu')
