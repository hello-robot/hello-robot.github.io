#!/usr/bin/env python

import click
from colorama import Style
import glob
import matplotlib.pyplot as plt
import stretch_body.hello_utils as hu
from os import makedirs
import yaml
from yaml import CDumper as Dumper

class TraceMgmt:
    """
    Manage trace data
    """
    def __init__(self,device):
        self.device=device
        self.device_name = device.name
        self.trace_directory = hu.get_stretch_directory() + 'log/trace_firmware'
        try:
            makedirs(self.trace_directory)
        except OSError:
            pass  # Exists

    def get_int(self,range,msg='value'):
        while True:
            result = input("Enter %s (range %d to %d):  "%(msg,range[0],range[1]))
            if result.isdigit() and range[0] <= int(result) <= range[1]:
                return int(result)
            print("Error Invalid Input")

    def get_trace_type(self,trace_data):
        if len(trace_data)==0:
            return None
        if len(trace_data[0]['status']):
            return 'status'
        if len(trace_data[0]['debug']):
            return 'debug'
        if len(trace_data[0]['print']):
            return 'print'

    def run_menu(self):
        trace_data=[]

        while True:
            if len(trace_data):
                tt=self.get_trace_type(trace_data)
                msg = 'Current trace: Device %s | Type: %s: ' % (self.device_name, tt.upper())
                if tt =='status' or tt=='print':
                    t0=trace_data[0][tt]['timestamp']
                    t1=trace_data[-1][tt]['timestamp']
                    msg =  msg+ '| Duration (s): %f | Start timestamp %f'%(t1-t0,t0)
                click.secho(msg, fg="green", bold=True)
            else:
                click.secho('No trace loaded', fg="yellow", bold=True)
            print('')
            print(Style.BRIGHT + '############### %s ################'%self.device_name.upper() + Style.RESET_ALL)
            print('Enter command. (q to quit)')
            print('r: record trace on device')
            print('d: load trace from device')
            print('l: load trace from file')
            print('s: save trace to file')
            print('y: display trace data')
            print('x: print trace to console')
            print('q: quit')
            print('-------------------------------------')
            #try:
            r = input()
            if r == 'q' or r == 'Q':
                return
            elif r=='r':
                trace_data=self.record_trace()
            elif r=='d':
                trace_data=self.load_trace_from_device()
            elif r=='l':
                trace_data=self.load_trace_from_file()
            elif r == 's':
                self.save_trace(trace_data)
            elif r == 'y':
                self.display_trace(trace_data)
            elif r== 'x':
                print(trace_data)
            else:
                self.device.pull_status()
                self.device.pretty_print()
            # except(TypeError, ValueError):
            #     print('Invalid entry')

    def record_trace(self):
        print('')
        dd=None

        input("Hit enter to begin recording")
        self.device.enable_firmware_trace()
        self.device.push_command()
        print('\nRecording...\n')
        input("Hit enter to end recording")
        self.device.disable_firmware_trace()
        self.device.push_command()
        print('Reading trace back from recording. This may take a minute...')
        trace_data = self.device.read_firmware_trace()
        if len(trace_data)==0:
            print('No trace data found for %s'%self.device_name)
        return trace_data

    def load_trace_from_file(self):
            # Retrieve sorted list of all trace files
            all_files = glob.glob(self.trace_directory + '/*.yaml')
            all_files.sort()
            if len(all_files):
                print('--- Firmware Trace Files ---')
                for i in range(len(all_files)):
                    print('%d: %s'%(i,all_files[i]))
                fn=all_files[self.get_int([0,len(all_files)-1],'FILE_ID')]
                ll=fn[fn.find('trace_fw_')+9:]
                device_name=ll[:ll.find('_')]
                with open(fn, 'r') as s:
                    return(yaml.load(s, Loader=yaml.FullLoader))
            else:
                print('No trace files available')
            return [],''
    def load_trace_from_device(self):
        print('')
        print('Reading trace from device. This may take a minute...')
        print('')
        trace_data = self.device.read_firmware_trace()
        if len(trace_data)==0:
            print('No trace data found for %s'%self.device_name)
        return trace_data

    def save_trace(self,trace_data):
        if len(trace_data)==0:
            print('No trace data to save')
            return

        time_string = hu.create_time_string()
        fn = self.trace_directory + '/trace_fw_' + self.device_name + '_' +time_string+'.yaml'
        print('Creating trace: %s'%fn)
        with open(fn, 'w+') as fh:
            fh.write('###%s###\n'%self.device_name)
            yaml.dump(trace_data, fh, encoding='utf-8', default_flow_style=False, Dumper=Dumper) #Use C YAML dumper for 5x speed increase over Python

    def display_trace(self,trace_data):
        tt=self.get_trace_type(trace_data)
        if tt=='status':
            self.do_plot_status(trace_data)
        if tt=='debug':
            self.do_plot_debug(trace_data)
        if tt=='print':
            self.do_plot_print(trace_data)
    def do_plot_print(self,trace_data):
        print(Style.BRIGHT + '############### Echoing Print Trace: %s ################'%self.device_name.upper() + Style.RESET_ALL)
        if len(trace_data[0]['print'])==0:
            print('No Print Trace data available')
        else:
            data = []
            for k in trace_data:
                print('%f: %s'%(k['print']['timestamp'],k['print']['line']))
                data.append(k['print']['x'])
            print('---------- PLOT DATA ----------')
            print(data)
            print('')

            plt.ion()  # enable interactivity
            fig, axes = plt.subplots(1, 1, figsize=(15.0, 8.0), sharex=True)
            if fig.canvas.manager is not None:
                fig.canvas.manager.set_window_title('TRACE %s | %s' % (self.device_name.upper(), field_name.upper()))
            axes.set_yscale('linear')
            axes.set_xlabel('Sample')
            axes.set_ylabel('X')
            axes.grid(True)
            axes.plot(data, 'b')
            fig.canvas.draw_idle()

    def do_plot_debug(self,trace_data):
        print(Style.BRIGHT + '############### Plotting Debug Trace: %s ################'%self.device_name.upper() + Style.RESET_ALL)
        print('----- Trace Fields -----')
        s0=trace_data[0]['debug']
        kk=list(s0.keys())
        if len(kk)==0:
            print('No data available')
            return
        kk.sort()
        field_keys=[]
        for k in kk:
            if type(s0[k])==int or type(s0[k])==float or type(s0[k])==bool:
                field_keys.append(k)
                print('%d: %s'%(len(field_keys)-1,str(k)))
        print('')
        field_name=field_keys[self.get_int([0,len(field_keys)-1],'FIELD ID')]
        data=[]
        for t in trace_data:
            data.append(t['debug'][field_name])
        print('---------- PLOT DATA ----------')
        print(data)
        print('')

        plt.ion()  # enable interactivity
        fig, axes = plt.subplots(1, 1, figsize=(15.0, 8.0), sharex=True)
        if fig.canvas.manager is not None:
            fig.canvas.manager.set_window_title('TRACE %s | %s' % (self.device_name.upper(), field_name.upper()))
        axes.set_yscale('linear')
        axes.set_xlabel('Sample')
        axes.set_ylabel(field_name.upper())
        axes.grid(True)
        axes.plot(data, 'b')
        fig.canvas.draw_idle()

    def do_plot_status(self,trace_data):
        print(Style.BRIGHT + '############### Plotting Status Trace: %s ################'%self.device_name.upper() + Style.RESET_ALL)
        print('----- Trace Fields -----')
        s0=trace_data[0]['status']
        kk=list(s0.keys())
        if len(kk)==0:
            print('No data available')
            return
        kk.sort()
        field_keys=[]
        for k in kk:
            if type(s0[k])==int or type(s0[k])==float or type(s0[k])==bool:
                field_keys.append(k)
                print('%d: %s'%(len(field_keys)-1,str(k)))
        print('')
        field_name=field_keys[self.get_int([0,len(field_keys)-1],'FIELD ID')]
        data=[]
        for t in trace_data:
            data.append(t['status'][field_name])
        print('---------- PLOT DATA ----------')
        print(data)
        print('')

        plt.ion()  # enable interactivity
        fig, axes = plt.subplots(1, 1, figsize=(15.0, 8.0), sharex=True)
        if fig.canvas.manager is not None:
            fig.canvas.manager.set_window_title('TRACE %s | %s' % (self.device_name.upper(), field_name.upper()))
        axes.set_yscale('linear')
        axes.set_xlabel('Sample')
        axes.set_ylabel(field_name.upper())
        axes.grid(True)
        axes.plot(data, 'b')
        fig.canvas.draw_idle()
