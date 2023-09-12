#!/usr/bin/env python
from threading import Thread, Lock
from subprocess import Popen, PIPE, STDOUT
import time
import os
import sys
import stretch_body.hello_utils as hu

hu.print_stretch_re_use()

class Dmesg_monitor:
    """
    Run live dmesg fetching in the backgorund using threading. Query the collected dmesg message
    outputs or clear them in between sessions. Save the collected dmesg output at the end.

    Params
    ------
    print_new_msg =  Prints Dmesg live if True
    log_fn = Optional file path to save the log at stop of dmesg monitor

    """

    def __init__(self, print_new_msg=False, log_fn=None):
        self.prev_out = None
        self.thread = None
        self.output_list = []
        self.is_live = False
        self.lock = Lock()
        self.print_new_msg = print_new_msg
        self.log_fn = log_fn
        os.system("sudo echo ''")

    def dmesg_fetch_clear(self):
        out = Popen("sudo dmesg -c", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True,
                    stderr=PIPE).stdout.read().decode("utf-8")
        out = str(out).split('\n')
        with self.lock:
            try:
                # Filter ghost lines
                if out[0] != self.prev_out[-1]:
                    for o in out:
                        if len(o) > 0:
                            self.output_list.append(o)
                            if self.print_new_msg:
                                print("[DMESG]...{}".format(o))
            except:
                pass
        self.prev_out = out

    def write_lines_to_file(self, lines, file_path):
        with open(os.path.expanduser(file_path), 'w') as file:
            for line in lines:
                file.write(line + '\n')
        print("DMESG Log saved to: {}".format(self.log_fn))

    def start(self):
        print("Starting DMESG capture....")
        self.is_live = True
        self.thread = Thread(target=self.live)
        self.thread.start()

    def stop(self):
        self.is_live = False
        self.thread.join()
        print("Ending DMESG capture....")
        self.save_log()

    def save_log(self):
        if self.log_fn is not None:
            self.write_lines_to_file(self.output_list, self.log_fn)

    def live(self):
        while self.is_live:
            self.dmesg_fetch_clear()

    def clear(self):
        print("Clearing the dmesg log buffer.")
        with self.lock:
            self.output_list = []

    def get_latest_msg(self):
        return self.prev_out

    def get_output_list(self):
        return self.output_list

fn = f'/tmp/dmesg_log_{int(time.time())}.log'
dmesg_monitor = Dmesg_monitor(print_new_msg=True,log_fn=fn)

def menu_top():
    print('------ MENU -------')
    print('m: menu')
    print('s: Save the log')
    print('c: Clear the log buffer')
    print('d: Delete old dmesg logs')
    print('q: Quit')
    print('-------------------')

def step_interaction():
    menu_top()
    x=sys.stdin.readline()
    if len(x)>1:
        if x[0]=='m':
            menu_top()
        if x[0]=='s':
            fn = f'/tmp/dmesg_log_{int(time.time())}.log'
            dmesg_monitor.log_fn = fn
            dmesg_monitor.save_log()
        if x[0]=='c':
            dmesg_monitor.clear()
        if x[0]=='d':
            print("Deleted all the previous logs from `/tmp/`")
            os.system('rm /tmp/dmesg_log_*')
        if x[0]=='q':
            print("Exiting....")
            sys.exit()



try:
    dmesg_monitor.start()
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit):
    dmesg_monitor.stop()