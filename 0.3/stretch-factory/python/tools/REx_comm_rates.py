#!/usr/bin/env python3
import argparse
import stretch_body.pimu
import stretch_body.wacc
import stretch_body.arm
import stretch_body.base
import stretch_body.lift
import stretch_body.head
import stretch_body.end_of_arm
import stretch_body.robot
import stretch_body.hello_utils as hu
import time
import click


hu.print_stretch_re_use()


r=stretch_body.robot.Robot()
r.startup()
click.secho('Measuring rates. This will take about 60s...', fg="yellow",bold=True)
print('')
# ########################################################################
ts = time.time()
nitr1=1000
nitr2=100
for i in range(nitr1):
    r.arm.move_by(0)
    r.lift.move_by(0)
    r.base.translate_by(0)
    r.pimu.set_fan_on()
    r.wacc.set_D2(0)
    r.push_command()
dt1 = time.time() - ts

s = r.get_status()
ts = time.time()
for i in range(nitr2):
    for m in r.head.motors:
        r.head.motors[m].move_to(s['head'][m]['pos'])
    for m in r.end_of_arm.motors:
        r.end_of_arm.motors[m].move_to(s['end_of_arm'][m]['pos'])
dt2 = time.time() - ts

# ########################################################################
click.secho(' Robot Threads '.center(75, '-'), fg="white", bold=True)
click.secho('%s | %s | %s | %s | %s' % ('Name'.ljust(25),'Average'.ljust(10),'Worst Case'.ljust(10),'Std Dev.'.ljust(10),'Target'.ljust(10)), fg="green", bold=True)
click.secho('%s | %s | %s | %s | %s' % ('non_dxl_thread'.upper().ljust(25),
                                   ('%.2f'%r.non_dxl_thread.stats.status['avg_rate_hz']).ljust(10),
                                   ('%.2f' % r.non_dxl_thread.stats.status['min_rate_hz']).ljust(10),
                                   ('%.2f'%r.non_dxl_thread.stats.status['std_rate_hz']).ljust(10),
                                   ('%.2f'%r.params['rates']['NonDXLStatusThread_Hz']).ljust(10)), fg="green", bold=False)

click.secho('%s | %s | %s | %s | %s' % ('dxl_head_thread'.upper().ljust(25),
                                   ('%.2f' % r.dxl_head_thread.stats.status['avg_rate_hz']).ljust(10),
                                   ('%.2f' % r.dxl_head_thread.stats.status['min_rate_hz']).ljust(10),
                                   ('%.2f' % r.dxl_head_thread.stats.status['std_rate_hz']).ljust(10),
                                   ('%.2f' % r.params['rates']['DXLStatusThread_Hz']).ljust(10)), fg="green",bold=False)

click.secho('%s | %s | %s | %s | %s' % ('dxl_end_of_arm_thread'.upper().ljust(25),
                                   ('%.2f' % r.dxl_end_of_arm_thread.stats.status['avg_rate_hz']).ljust(10),
                                   ('%.2f' % r.dxl_end_of_arm_thread.stats.status['min_rate_hz']).ljust(10),
                                   ('%.2f' % r.dxl_end_of_arm_thread.stats.status['std_rate_hz']).ljust(10),
                                   ('%.2f' % r.params['rates']['DXLStatusThread_Hz']).ljust(10)), fg="green",bold=False)

click.secho('%s | %s | %s | %s | %s' % ('sys_thread'.upper().ljust(25),
                                   ('%.2f' % r.sys_thread.stats.status['avg_rate_hz']).ljust(10),
                                   ('%.2f' % r.sys_thread.stats.status['min_rate_hz']).ljust(10),
                                   ('%.2f' % r.sys_thread.stats.status['std_rate_hz']).ljust(10),
                                   ('%.2f' % r.params['rates']['SystemMonitorThread_Hz']).ljust(10)), fg="green",bold=False)

r.stop()
# ########################################################################
print('')
click.secho('Device Comms '.center(75, '-'), fg="white", bold=True)
print('Robot non-DXL push:\t{:.2f}Hz'.format(nitr1 / dt1))
print('Robot DXL push:\t\t{:.2f}Hz'.format(nitr2 / dt2))

p=stretch_body.pimu.Pimu()
p.startup()
ts = time.time()
for i in range(nitr1):
    p.set_fan_off()
    p.push_command()
    p.pull_status()
dt = time.time() - ts
p.stop()
print('Pimu push-pull:\t\t{:.2f}Hz'.format(nitr1 / dt))

w=stretch_body.wacc.Wacc()
w.startup()
ts = time.time()
for i in range(nitr1):
    w.set_D3(0)
    w.push_command()
    w.pull_status()
dt = time.time() - ts
w.stop()
print('Wacc push-pull:\t\t{:.2f}Hz'.format(nitr1 / dt))

a=stretch_body.arm.Arm()
a.startup()
ts = time.time()
for i in range(nitr1):
    a.move_by(0)
    a.push_command()
    a.pull_status()
dt = time.time() - ts
a.stop()
print('Arm push-pull:\t\t{:.2f}Hz'.format(nitr1 / dt))

l=stretch_body.lift.Lift()
l.startup()
ts = time.time()
for i in range(nitr1):
    l.move_by(0)
    l.push_command()
    l.pull_status()
dt = time.time() - ts
l.stop()
print('Lift push-pull:\t\t{:.2f}Hz'.format(nitr1 / dt))

b=stretch_body.base.Base()
b.startup()
ts = time.time()
for i in range(nitr1):
    b.translate_by(0)
    b.left_wheel.push_command()
    b.left_wheel.pull_status()
dt = time.time() - ts
print('Left Wheel push-pull:\t{:.2f}Hz'.format(nitr1 / dt))

ts = time.time()
for i in range(nitr1):
    b.translate_by(0)
    b.right_wheel.push_command()
    b.right_wheel.pull_status()
dt = time.time() - ts
b.stop()
print('Right Wheel push-pull:\t{:.2f}Hz'.format(nitr1 / dt))

h = stretch_body.head.Head()
h.startup()
ts = time.time()
h.pull_status()
for i in range(nitr2):
    for m in h.motors:
        h.motors[m].move_to(h.motors[m].status['pos'])
    h.pull_status()
dt = time.time() - ts
h.stop()
print('Head push-pull:\t\t{:.2f}fHz'.format(nitr2 / dt))

e = stretch_body.end_of_arm.EndOfArm()
e.startup()
ts = time.time()
for i in range(nitr2):
    for m in e.motors:
        e.motors[m].move_to(e.motors[m].status['pos'])
    e.pull_status()
dt = time.time() - ts
e.stop()
print('Endof Arm push-pull:\t{:.2f}fHz'.format(nitr2 / dt))

print('')