#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.scope
from stretch_body.hello_utils import *
from matplotlib.widgets import Slider, Button
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import time
import argparse
from stretch_body.hello_utils import *

parser=argparse.ArgumentParser(description='Tune stepper controller gains using a GUI')
parser.add_argument("--motion", help="Motion setting (e.g. default, fast, slow)", type=str, default='default',choices=['slow','default','fast','max'])
parser.add_argument("--sync", help="Use motor sync", action="store_true")

group1 = parser.add_mutually_exclusive_group(required=True)
group1.add_argument("--arm", help="Test trajectories on the arm joint", action="store_true")
group1.add_argument("--lift", help="Test trajectories on the lift joint", action="store_true")
group1.add_argument("--base_translate", help="Test trajectories on base translation", action="store_true")
group1.add_argument("--base_rotate", help="Test trajectories on base rotation", action="store_true")
group2= parser.add_mutually_exclusive_group(required=True)
group2.add_argument("--pos_traj", help="Tune Postion Trajectory Controller", action="store_true")
group2.add_argument("--pos_pid", help="Tune Position PID Controller", action="store_true")



args, _ = parser.parse_known_args()


# ################### Gui ###################################

class StepperCtrlTuningGui:

    def __init__(self, x, y, g,motors, yrange=None, sense_frequency=100,
                 title='Interactive Scope', xlabel='Time (s)', ylabel='Data'):
        """Interactive scope for controller tuning.

        Allows interactive cubic trajectory planning using waypoints and sliders.
        Execution and reset buttons start and stop trajectory tracking.
        While trajectory executes, joint position stream is plotted over the
        planned stream. Trajectory can be modified while being executed.

        Parameters
        ----------
        x : list(float)
            Starting x axis waypoints. Generally is time (seconds)
        y : list(float)
            Starting y axis waypoints. Generally is joint position (unitless)

        g: dict: Gains for controller

        j: stepper joint
        yrange : tuple(float, float)
            Two tuple representing joint limits
        sense_frequency : int
            Frequency (hz) at which scope measures joint position
        title : str
            Title of the scope window
        xlabel: str
            Label for the x axis
        ylable: str
            Label for the y axis

        Attributes
        ----------
        initx, inity, initv : list(float)
            initial waypoints for resetting graph
        sensex, sensey : list(float)
            used to plot measured joint position over time
        epsilon : int
            clicking radius for waypoint
        """
        self.x, self.initx = x[:], x[:]
        self.y, self.inity = y[:], y[:]
        self.g=g.copy()
        self.motors=motors
        self.sensex = []
        self.sensey = []
        self.num_gains = len(list(self.g.keys()))
        self.yrange = yrange if yrange is not None else (0, 10)
        self.epsilon = 15
        self.pind = None

        # Setup plot and widgets
        widget_loc = lambda x: plt.axes([0.84, 0.8-((x)*0.05), 0.12, 0.02])
        self.fig, self.axes = plt.subplots(1, 1, figsize=(20.0, 8.0), sharex=True)
        self.fig.subplots_adjust(right=0.8)
        self.fig.canvas.set_window_title(title)
        self.axes.set_yscale('linear')
        dy=max(self.yrange)-min(self.yrange)
        self.axes.set_ylim(min(self.yrange) - dy*0.25, max(self.yrange) + dy*0.25)
        self.axes.axhspan(min(self.yrange) - 2 ** 32, min(self.yrange), facecolor='0.2', alpha=0.5)
        self.axes.axhspan(max(self.yrange), max(self.yrange) + 2 ** 32, facecolor='0.2', alpha=0.5)
        self.axes.set_xlabel(xlabel)
        self.axes.set_ylabel(ylabel)
        self.axes.grid(True)
        self.reset_button = Button(widget_loc(self.num_gains), 'Reset')
        self.reset_button.on_clicked(self._reset)
        self.exec_button = Button(widget_loc(self.num_gains + 1), 'Execute', color='limegreen', hovercolor='lightgreen')
        self.exec_button.on_clicked(self._execute)
        self.save_button = Button(widget_loc(self.num_gains+2), 'Save')
        self.save_button.on_clicked(self._save)
        self.sliders = []
        k=list(self.g.keys())
        for i in range(self.num_gains):
            gg=g[k[i]]
            if gg['init_val']==0:
                gg['min_val']=-0.5
                gg['max_val']=0.5
            s = Slider(widget_loc(i), '{0}'.format(k[i]), gg['min_val'], gg['max_val'], valinit=gg['init_val'])
            s.on_changed(self._update)
            self.sliders.append(s)
        self.fig.canvas.mpl_connect('button_press_event', self._mouse_down_cb)
        self.fig.canvas.mpl_connect('button_release_event', self._mouse_up_cb)
        self.fig.canvas.mpl_connect('motion_notify_event', self._mouse_move_cb)
        self.anim = animation.FuncAnimation(self.fig, self._animate, interval=int(1000 / sense_frequency))
        self.executing = False

        # Plot data

        self.m, = self.axes.plot(self.initx, self.inity, color='gray', linestyle='--', label='Trajectory')
        self.s, = self.axes.plot(self.sensex, self.sensey, color='limegreen', label='Sense')
        self.l, = self.axes.plot(self.initx, self.inity, color='tab:blue', linestyle='none', marker='o', markersize=12)
        self.lfirst, = self.axes.plot(self.initx[0], self.inity[0], color='0.6', linestyle='none', marker='o', markersize=12)
        self.axes.legend(loc="upper right")
        self._update(self)

    def start(self, exec_func, sense_func, setpoints_change_func, stop_func):
        """Starts the scope with four callback functions

        Parameters
        ----------
        exec_func : func
            Called when execute button in scope is pressed
        sense_func : func
            Called at ``sense_frequency`` to plot joint position
        setpoints_change_func : func
            Called with updated waypoints when user changes them
        stop_func : func
            Called when stop button in scope is pressed
        """
        self.exec_func = exec_func
        self.sense_func = sense_func
        self.setpoints_change_func = setpoints_change_func
        self.stop_func = stop_func
        plt.show()

    def _animate(self, i):
        sense = self.sense_func()
        if sense:
            self.sensex.append(sense[0])
            self.sensey.append(sense[1])
            self._update(self)

    def _execute(self, e):
        if not self.executing:
            self.exec_func(self.x, self.y)
            self.anim.event_source.start()
            self.executing = True

    def _save(self,e):
        print('Saving gains')
        # for k in self.g.keys():
        #     key = self.motors.motor.name + '.gains.'+str(k)
        #     print('Writing: %s with %f'%(key,self.g[k]['val']))
        #     j.write_user_param_to_YAML(key,float(self.g[k]['val']))

    def _reset(self, e):
        self.anim.event_source.stop()
        stopped_pos = self.stop_func()
        self.inity[0] = stopped_pos
        self.executing = False
        self.sensex = []
        self.sensey = []
        self.x = self.initx[:]
        self.y = self.inity[:]
        # k = list(self.g.keys())
        # for i in range(len(k)):
        #     self.sliders[i].set_val(self.g[k[i]]['init_val'])
        self._update(self)

    def _update(self, e):

        change_gains=False
        k=list(self.g.keys())
        for i in range(len(k)):
            if self.g[k[i]]['val']!=self.sliders[i].val:
                change_gains=True
            self.g[k[i]]['val']=self.sliders[i].val
        for m in self.motors:
            if change_gains:
                for i in range(len(k)):
                    gain_name=k[i]
                    m.gains[gain_name]=self.g[gain_name]['val']
                m.set_gains(m.gains)
                m.push_command()
        if not self.executing and (self.pind is not None or isinstance(e, np.float64)):
            self.setpoints_change_func(self.x, self.y)
        self.lfirst.set_xdata(self.x[0])
        self.lfirst.set_ydata(self.y[0])

        self.l.set_xdata(self.x)
        self.l.set_ydata(self.y)
        self.s.set_xdata(self.sensex)
        self.s.set_ydata(self.sensey)
        # splinex = []
        # spliney = []
        # a = list(zip(self.x, self.y, self.v))
        # for (i, f) in zip(a, a[1:]):
        #     seg = generate_cubic_polynomial(i, f)
        #     segx = np.arange(i[0], f[0], 0.05)
        #     for t in segx:
        #         splinex.append(t)
        #         spliney.append(evaluate_polynomial_at(seg[1:], t - i[0])[0])
        self.m.set_xdata(self.x)
        self.m.set_ydata(self.y)
        self.fig.canvas.draw_idle()

    def _mouse_down_cb(self, e):
        if e.inaxes is None:
            return
        if e.button != 1:
            return
        t = self.axes.transData.inverted()
        tinv = self.axes.transData
        xy = t.transform([e.x, e.y])
        xr = np.reshape(self.x, (np.shape(self.x)[0], 1))
        yr = np.reshape(self.y, (np.shape(self.y)[0], 1))
        xy_vals = np.append(xr, yr, 1)
        xyt = tinv.transform(xy_vals)
        xt, yt = xyt[:, 0], xyt[:, 1]
        d = np.hypot(xt - e.x, yt - e.y)
        indseq, = np.nonzero(d == d.min())
        ind = indseq[0]
        if d[ind] >= self.epsilon:
            ind = None
        self.pind = ind

    def _mouse_up_cb(self, e):
        if e.button != 1:
            return
        self.pind = None

    def _mouse_move_cb(self, e):
        if self.pind is None:
            return
        if e.inaxes is None:
            return
        if e.button != 1:
            return
        if self.pind == 0:
            return
        if self.pind == len(self.x) - 1:
            prelimit = self.x[self.pind - 1] + 0.5
            postlimit = self.x[self.pind] + 10000
        else:
            prelimit = self.x[self.pind - 1] + 0.5
            postlimit = self.x[self.pind + 1] - 0.5

        if e.xdata > prelimit and e.xdata < postlimit:
            self.x[self.pind] = e.xdata
        if e.ydata > min(self.yrange) and e.ydata < max(self.yrange):
            self.y[self.pind] = e.ydata
        self._update(self)


# ################### ARM ###################################
is_motion_active=False
motion_ts = time.time()
cmd_start_ts=time.time()
cmd_end_ts=time.time()
setpoints_x=[]
setpoints_y=[]
current_setpoint_idx=0
dirty_printer=True
gains_list=[]
v_m=0
a_m=0
if args.pos_traj:
    gains_list = ['pKp_d', 'pKi_d', 'pKd_d', 'pKi_limit', 'pLPF']
if args.pos_pid:
    gains_list = ['pKp_d', 'pKi_d', 'pKd_d', 'pKi_limit', 'pLPF']



if args.base_translate or args.base_rotate:
    import stretch_body.base
    j = stretch_body.base.Base()
    j.startup(threaded=False)
    if not args.sync:
        j.left_wheel.disable_sync_mode()
        j.right_wheel.disable_sync_mode()
    else:
        import stretch_body.pimu
        p=stretch_body.pimu.Pimu()
        p.startup()

    j.left_wheel.disable_guarded_mode()
    j.right_wheel.disable_guarded_mode()
    j.push_command()
    if args.base_translate:
        if args.motion=='slow':
            setpoints_x= [0.0, 6.0, 12.0]
            setpoints_y = [0.0, 0.1, 0.0]
            j_yrange = (-0.15, 0.15)
        else:
            setpoints_x = [0.0, 3.0, 6.0]
            setpoints_y = [0.0, 0.1, 0.0]
            j_yrange = (-0.15, 0.15)
        j_title = 'Stretch Base Translate Tuning'
        j_label = "Stretch Base Translate Range (m)"
    if args.base_rotate:
        if args.motion=='slow':
            setpoints_x= [0.0, 6.0, 12.0]
            setpoints_y = [0.0, 1.0, 0.0]
            j_yrange = (-1.15, 1.15)
        else:
            setpoints_x = [0.0, 3.0, 6.0]
            setpoints_y = [0.0, 1.0, 0.0]
            j_yrange = (-1.15, 115)
        j_title = 'Stretch Base Rotate Tuning'
        j_label = "Stretch Base Rotate Range (rad)"
    j_req_calibration = True
    v_m = j.params['motion'][args.motion]['vel_m']
    a_m = j.params['motion'][args.motion]['accel_m']
    setpoint_odom=0.0
    gains = {}
    for g in gains_list:
        gains[g] = {}
        gains[g]['init_val'] = j.left_wheel.gains[g]
        gains[g]['min_val'] = j.left_wheel.gains[g] - abs(j.left_wheel.gains[g])
        gains[g]['max_val'] = j.left_wheel.gains[g] + abs(j.left_wheel.gains[g])
        gains[g]['val'] = j.left_wheel.gains[g]

if args.arm:
    import stretch_body.arm
    j = stretch_body.arm.Arm()
    j.startup(threaded=False)
    j.motor.disable_sync_mode()
    j.motor.disable_guarded_mode()
    j.push_command()
    setpoints_x= [0.0, 3.0, 6.0]
    setpoints_y = [0.1, 0.4, 0.1]
    j_yrange = (j.params['range_m'][0], j.params['range_m'][1])
    j_title='Stretch Arm Tuning'
    j_label="Stretch Arm Joint Range (m)"
    j_req_calibration = True
    v_m = j.params['motion'][args.motion]['vel_m']
    a_m = j.params['motion'][args.motion]['accel_m']

    gains = {}
    for g in gains_list:
        gains[g] = {}
        gains[g]['init_val'] = j.motor.gains[g]
        gains[g]['min_val'] = j.motor.gains[g] - abs(j.motor.gains[g])
        gains[g]['max_val'] = j.motor.gains[g] + abs(j.motor.gains[g])
        gains[g]['val'] = j.motor.gains[g]


if args.lift:
    import stretch_body.lift
    j = stretch_body.lift.Lift()
    j.startup(threaded=False)
    j.motor.disable_sync_mode()
    #j.motor.disable_guarded_mode()
    j.push_command()
    setpoints_x= [0.0, 5.0, 10.0]
    setpoints_y = [0.4, 0.6, 0.4]
    j_yrange = (j.params['range_m'][0], j.params['range_m'][1])
    j_title='Stretch Lift Tuning'
    j_label="Stretch Lift Joint Range (m)"
    j_req_calibration = True
    v_m = j.params['motion'][args.motion]['vel_m']
    a_m = j.params['motion'][args.motion]['accel_m']

    gains={}
    for g in gains_list:
        gains[g]={}
        gains[g]['init_val']=j.motor.gains[g]
        gains[g]['min_val'] = j.motor.gains[g]-abs(j.motor.gains[g])
        gains[g]['max_val'] = j.motor.gains[g]+abs(j.motor.gains[g])
        gains[g]['val'] = j.motor.gains[g]
# ##########################################################################

def start_trajectory(times, positions):
    global motion_ts, setpoints_x, setpoints_y, is_motion_active,current_setpoint_idx

    setpoints_x = times
    setpoints_y = positions


    j.pull_status()
    if args.base_translate or args.base_rotate:
        pass
    else:
        j.move_to(setpoints_y[0])
        j.push_command()
        j.motor.wait_until_at_setpoint(timeout=5)
    #add_waypoint_cb(times,positions,velocities)
    motion_ts = time.time()
    is_motion_active=True
    current_setpoint_idx=0


def sense_trajectory():
    j.pull_status()
    global motion_ts, setpoints_x, setpoints_y, is_motion_active,current_setpoint_idx, cmd_end_ts, cmd_start_ts,dirty_printer
    if is_motion_active:
        if args.base_translate or args.base_rotate:
            err=rad_to_deg(j.left_wheel.status['err'])
            ii = j.left_wheel.status['current']
            ef=j.left_wheel.status['effort_pct']
            dbg=j.left_wheel.status['debug']
        else:
            err=rad_to_deg(j.motor.status['err'])
            ii = j.motor.status['current']
            ef=j.motor.status['effort_pct']
            dbg = j.motor.status['debug']
        dt = time.time() - motion_ts
        print('Current: %f  Effort: %f Error (deg) %f Debug %f'%(ii,ef,err,dbg))


        if args.pos_pid and not args.base_translate:
            if current_setpoint_idx > 0 and j.motor.status['near_pos_setpoint'] and dirty_printer:
                cmd_end_ts = time.time()
                dur = cmd_end_ts - cmd_start_ts
                dx = setpoints_y[current_setpoint_idx] - setpoints_y[current_setpoint_idx - 1]
                print('Setpoint %d. Duration %f. Traveled: %f. Velocity %f' % (current_setpoint_idx, dur, dx, dx / dur))
                dirty_printer = False
        if dt>setpoints_x[current_setpoint_idx] and current_setpoint_idx<len(setpoints_y)-1:
            dirty_printer=True
            current_setpoint_idx=current_setpoint_idx+1
            #print('Moving to',setpoints_y[current_setpoint_idx])
            #
            if args.base_translate or args.base_rotate:
                if args.pos_pid:
                    pass
                if (args.pos_traj ) and current_setpoint_idx>0:
                    inc_x = setpoints_y[current_setpoint_idx] - setpoints_y[current_setpoint_idx - 1]
                    if args.base_translate:
                        j.translate_by(inc_x, v_m=v_m, a_m=a_m)
                    if args.base_rotate:
                        j.rotate_by(inc_x, v_r=v_m, a_r=a_m)
            else:
                if args.pos_pid:
                    j.motor.set_command(mode=j.motor.MODE_POS_PID, x_des=j.translate_m_to_motor_rad(setpoints_y[current_setpoint_idx]))
                if args.pos_traj:
                    j.move_to(setpoints_y[current_setpoint_idx],v_m=v_m, a_m=a_m)

            cmd_start_ts=time.time()
            j.push_command()
            if args.sync:
                p.trigger_motor_sync()
        if dt>setpoints_x[-1]:
            is_motion_active=False
            stop_trajectory()
        if args.base_translate:
            return (time.time() - motion_ts, j.status['x'])
        elif args.base_rotate:
            return (time.time() - motion_ts, j.status['theta'])
        else:
            return (time.time()-motion_ts, j.status['pos'])
    if args.base_translate:
        return (0,j.status['x'])
    elif args.base_rotate:
        return (0,j.status['theta'])
    else:
        return (0,j.status['pos'])

def update_trajectory(times, positions):
    global motion_ts, setpoints_x, setpoints_y, is_motion_active, current_setpoint_idx
    setpoints_x=times
    setpoints_y=positions


def stop_trajectory():
    #j.motor.enable_safety()
    #j.push_command()
    if args.base_translate or args.base_rotate:
        j.first_step=True
        return 0
    else:
        return j.status['pos']

if args.base_translate or args.base_rotate:
    s = StepperCtrlTuningGui(setpoints_x, setpoints_y,gains,[j.left_wheel, j.right_wheel],yrange=j_yrange,sense_frequency=20,title=j_title, ylabel=j_label)
else:
    s = StepperCtrlTuningGui(setpoints_x, setpoints_y,gains,[j.motor],yrange=j_yrange,sense_frequency=20,title=j_title, ylabel=j_label)

s.start(start_trajectory, sense_trajectory, update_trajectory, stop_trajectory)
