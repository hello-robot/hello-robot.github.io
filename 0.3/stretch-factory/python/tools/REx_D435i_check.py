#!/usr/bin/env python
import argparse
#from concurrent.futures import thread
import os
import sys
from subprocess import Popen, PIPE, STDOUT
from colorama import Fore, Back, Style
import stretch_factory.hello_device_utils as hdu
from threading import Thread
import time
import signal
import numpy as np
from tabulate import tabulate
import stretch_body.robot

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='Tests the D435i stream and produces a check log that can be used for troubleshooting.')
parser.add_argument('-f', metavar='check_log_path', type=str, help='The path to save D435i Check Log (default:/tmp/d435i_check_log.txt)')
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--usb", help="Test USB version", action="store_true")
group.add_argument("--rate", help="Test data capture rate", action="store_true")
group.add_argument("--scan_head", help="Test data capture rate with Head Pan and Tilt to extremity", action="store_true")


args = parser.parse_args()
thread_stop = False
check_log = []
dmesg_log = []
log_file_path = "/tmp/d435i_check_log.txt"

# Include all the known kernel non problematic messages here
# index     1:no.occured 2:no.Acceptable_Occurances 3:Message
known_msgs=[[0,15,'uvcvideo: Failed to query (GET_CUR) UVC control'],
            [0,4,'Non-zero status (-71) in video completion handler'],
            [0,4,'No report with id 0xffffffff found'],
            [0,10,'uvcvideo: Found UVC 1.50 device Intel(R) RealSense(TM) Depth Camera 435'],
            [0,5,'uvcvideo: Unable to create debugfs'],
            [0,4,'hid-sensor-hub'],
            [0,6,'input: Intel(R) RealSense(TM) Depth Ca'],
            [0,1,'uvcvideo: Failed to resubmit video URB (-1).'],
            [0,1,'Netfilter messages via NETLINK v0.30.']]

# Enter each stream's pass fps rate
streams_assert = {'Depth':29,'Color':29,'Gyro':199,'Accel':62} # Refer create_config_target_*

pan_tilt_pos = (None,None)
usbtop_cmd = None

def create_config_target_hi_res():
    global check_log
    f = open('/tmp/d435i_confg.cfg', "w+")
    config_script = ["DEPTH,1280,720,30,Z16,0",
                     "COLOR,1920,1080,30,RGB8,0",
                     "ACCEL,1,1,63,MOTION_XYZ32F",
                     "GYRO,1,1,200,MOTION_XYZ32F"]

    header = ["STREAM","WIDTH","HEIGHT","FPS","FORMAT","STREAM_INDEX"]
    config_data = []
    check_log.append("Camera Stream Config:")
    for ll in config_script:
        f.write(ll+"\n")
        config_data.append(ll.split(','))
    config_table = str(tabulate(config_data,headers=header,tablefmt='github')).split('\n')
    for ll in config_table:
        check_log.append(ll)
    check_log.append('\n')

    f.close()
    target = {'duration': 31,
              'nframe': 900,
              'margin': 16,
              'streams':{
              'Color': {'target': 900, 'sampled': 0},
              'Depth': {'target': 900, 'sampled': 0},
              'Accel': {'target': 900, 'sampled': 0},
              'Gyro': {'target': 900, 'sampled': 0}}}
    return target

def create_config_target_low_res():
    global check_log
    f = open('/tmp/d435i_confg.cfg', "w+")
    config_script = ["DEPTH,424,240,30,Z16,0",
                     "COLOR,424,240,30,RGB8,0",
                     "ACCEL,1,1,63,MOTION_XYZ32F",
                     "GYRO,1,1,200,MOTION_XYZ32F"]

    header = ["STREAM","WIDTH","HEIGHT","FPS","FORMAT","STREAM_INDEX"]
    config_data = []
    check_log.append("Camera Stream Config:")
    for ll in config_script:
        f.write(ll+"\n")
        config_data.append(ll.split(','))
    config_table = str(tabulate(config_data,headers=header,tablefmt='github')).split('\n')
    for ll in config_table:
        check_log.append(ll)
    check_log.append('\n')

    f.close()
    target = {'duration': 31,
              'nframe': 900,
              'margin': 16,
              'streams':{
              'Color': {'target': 900, 'sampled': 0},
              'Depth': {'target': 900, 'sampled': 0},
              'Accel': {'target': 900, 'sampled': 0},
              'Gyro': {'target': 900, 'sampled': 0}}}
    return target

def check_install_v4l2():
    global check_log
    out = Popen("which v4l2-ctl", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()
    if len(out):
        out = Popen("v4l2-ctl --all | grep -A 3 -i 'uvcvideo'", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()
        check_log.append(out)
        print(out)
    else:
        print('"v4l2-utils" tool is required to be installed for logging USB video driver info.')
        x = raw_input('Enter "y" to proceed with Installation of "v4l2-utils".\n')
        if x=='y' or x=='Y':
            print('Installing v4l2-utils tool.....')
            script = 'sudo apt-get install -y v4l-utils'
            os.system(script)
            check = Popen("which v4l2-ctl", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()
            if check:
                print(Fore.GREEN +'[Pass] "v4l2-utils" sucessfully installed'+Style.RESET_ALL+'\n\n')
                out = Popen("v4l2-ctl --all | grep -A 3 -i 'uvcvideo'", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()
                check_log.append(out)
                print(out)
            else:
                print(Fore.RED + '[Fail] "v4l2-utils" did not install.'+Style.RESET_ALL)
        else:
            print(Fore.YELLOW+'[Warning] Skip logging USB Video Drivers.'+Style.RESET_ALL)

def get_usb_busID():
    """
    Search for Realsense D435i in the USB bus
    Gets the USB Bus number and device ID for monitoring
    Execute it at Start
    """
    global usbtop_cmd, check_log
    print('Starting D435i Check')
    print('====================')
    print('Searching for Realsense D435i in USB Bus...')
    out = Popen("usb-devices | grep -B 5 -i 'RealSense' | grep -i 'Bus'", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read()
    if(len(out)):
        out_list = out.split(' ')
        bus_no = None
        dev_id = None
        usb_found = False
        for i in range(len(out_list)):
            if out_list[i].find('Bus')!=-1:
                bus_no = out_list[i].split('=')[1]
                bus_no = int(bus_no)
            if out_list[i]=='Dev#=':
                dev_id = int(out_list[i+2])

        print(Fore.GREEN + '[Pass] Realsense D435i found at USB Bus_No : %d | Device ID : %d'%(bus_no,dev_id)+Style.RESET_ALL)
        check_log.append('[Pass] Realsense D435i found at USB Bus_No : %d | Device ID : %d'%(bus_no,dev_id))

        usbtop_cmd = "sudo usbtop --bus usbmon%d | grep 'Device ID %d' > /tmp/usbrate.txt"%(bus_no,dev_id)
        get_driver_versions()



    else:
        print(Fore.RED + '[Fail] Realsense D435i not found at USB Bus'+Style.RESET_ALL)
        check_log.append('[Fail] Realsense D435i not found at USB Bus')
        sys.exit()         

def check_usb():
    global check_log
    out = Popen("rs-enumerate-devices| grep Usb | grep 3.2", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read()
    if len(out):
        print(Fore.GREEN +'[Pass] Confirmed USB 3.2 connection to device'+Style.RESET_ALL)
        check_log.append('[Pass] Confirmed USB 3.2 connection to device')
    else:
        print(Fore.RED +'[Fail] Did not find USB 3.2 connection to device'+Style.RESET_ALL)
        check_log.append('[Fail] Did not find USB 3.2 connection to device')

def get_driver_versions():
    global check_log
    fw_details = Popen("rs-fw-update -l | grep -i 'firmware'", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read()
    fw_details = fw_details.split(',')[3]
    fw_version = fw_details.split(' ')[-1]
    print('D435i Firmware version: %s'%(fw_version))
    nuc_bios_version = Popen("sudo dmidecode -s bios-version", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read()
    system_version = Popen("sudo dmidecode -s system-version", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read()
    baseboard_version = Popen("sudo dmidecode -s baseboard-version", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read()
    processor_version = Popen("sudo dmidecode -s processor-version", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read()
    kernel_version = Popen("uname -r", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read()
    check_install_v4l2()
    check_log.append('\nD435i Firmware version: %s\n'%(fw_version))
    check_log.append("Linux Kernel Version : %s"%(kernel_version))
    check_log.append("NUC Bios Version : %s"%(nuc_bios_version))
    check_log.append("NUC System Version : %s"%(system_version))
    check_log.append("NUC Baseboard Version : %s"%(baseboard_version))
    check_log.append("Processor Version : %s"%(processor_version))
    
def check_ros():
    global check_log
    ros_out = Popen("rostopic list", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True,stderr=PIPE).stdout.read()
    if ros_out:
        print(Fore.YELLOW+'[Warning] roscore is running in background. Recommended to stop roscore.'+Style.RESET_ALL)
        check_log.append('\nroscore is running in background')

def get_frame_id_from_log_line(stream_type,line):
    if line.find(stream_type)!=0:
        return None
    return int(line.split(',')[2])

def check_frames_collected(data,target):
    global check_log
    check_log.append('\nRate Check Results......\n')
    for ll in data:
        for kk in target['streams'].keys():
            id=get_frame_id_from_log_line(kk,ll)
            if id is not None:
                target['streams'][kk]['sampled']=max(id,target['streams'][kk]['sampled'])
    for kk in target['streams'].keys():
        sampled_frames=target['streams'][kk]['sampled']
        min_frames=target['streams'][kk]['target']-target['margin']
        if sampled_frames>=min_frames:
            print(Fore.GREEN + '[Pass] Stream: %s with %d frames collected'%(kk,sampled_frames))
            check_log.append('[Pass] Stream: %s with %d frames collected'%(kk,sampled_frames))
        else:
            print(Fore.RED + '[Fail] Stream: %s with %d frames of %d collected'%(kk,sampled_frames,min_frames))
            check_log.append('[Fail] Stream: %s with %d frames of %d collected'%(kk,sampled_frames,min_frames))
    print(Style.RESET_ALL)

def check_FPS(data):
    global check_log
    for s in streams_assert.keys():
        fps = get_fps(data,s,'0')
        if fps>streams_assert[s]:
            print(Fore.GREEN+'[Pass] %s Rate : %f FPS'%(s,fps)+Style.RESET_ALL)
            check_log.append('[Pass] %s Rate : %f FPS'%(s,fps))
        else:
            print(Fore.RED+'[Fail] %s Rate : %f FPS < %d FPS'%(s,fps,streams_assert[s])+Style.RESET_ALL)
            check_log.append('[Fail] %s Rate : %f FPS < %d FPS'%(s,fps,streams_assert[s]))
    print('\n')

def get_fps(data,stream,t):
    timestamps = []
    for ll in data:
        tag = stream+','+t
        if tag in ll:
            l = ll.split(',')[-1].split('\n')[0]
            if stream=='Accel' or stream=='Gyro':
                l = ll.split(',')[-4].split('\n')[0]
            timestamp = float(l)/1000
            timestamps.append(timestamp)
    if len(timestamps)>1:
        duration = timestamps[-1]-timestamps[0]
        avg_fps = len(timestamps)/duration
        return avg_fps
    else:
        return 000.0

def check_dmesg(msgs):
    global known_msgs
    print('\nDMESG Issues.....')
    unknown_msgs=[]
    no_error=True
    for m in msgs:
        if len(m):
            found=False
            for i in range(len(known_msgs)):
                if m.find(known_msgs[i][2])!=-1:
                    found=True
                    known_msgs[i][0]=known_msgs[i][0]+1
            if not found:
                unknown_msgs.append(m)
    for i in range(len(known_msgs)):
        if known_msgs[i][0]>=known_msgs[i][1]:
            print(Fore.YELLOW+'[Warning] Excessive dmesg warnings (%d) of: %s'%(known_msgs[i][0],known_msgs[i][2]))
            no_error=False
    if len(unknown_msgs):
        print('[Warning] Unexpected dmesg warnings (%d)'%len(unknown_msgs))
        no_error=False
        for i in unknown_msgs:
            print(i)
    if no_error:
        print(Fore.GREEN+'[Pass] No unexpected dmesg warnings')
    print(Style.RESET_ALL)

def check_dmesg_thread():
    global thread_stop, check_log, pan_tilt_pos, dmesg_log
    print('\nMonitoring the DMESG Buffer for issues while collecting camera stream.\n\n')
    while thread_stop==False:
        out = hdu.exec_process(['sudo', 'dmesg', '-c'], True).split('\n')
        filtered_out = []
        filters = ['uvc','usb','input','hid']
        for o in out:
            for f in filters:
                if f in o:
                    filtered_out.append(o)
                    break
        if len(filtered_out)>0:
            for mesg in filtered_out:
                if len(mesg)>0:
                    if pan_tilt_pos[0]:
                        check_log.append(mesg+'   (Pan, Tilt)='+str(pan_tilt_pos))
                        dmesg_log.append(mesg)
                    else:
                        check_log.append(mesg)
                        dmesg_log.append(mesg)

def check_throughput(usbrate_file):
    global check_log
    ff = open(usbrate_file)
    data = ff.readlines()
    ff.close()
    hdu.exec_process(['sudo', 'rm', '/tmp/usbrate.txt'], True)
    in_speed_list = []
    out_speed_list = []
    avg_in_speed = 0.000
    max_in_speed = 0.000
    avg_out_speed = 0.000
    max_out_speed = 0.000
    for ll in data:
        line = ll.split('\t')
        if(len(line)==5):

            try:
                in_speed = float(line[3].split(' ')[0]) #Kib/s
                in_speed_list.append(in_speed)
            except:
                None
            try:
                out_speed = float(line[4].split(' ')[0]) #Kib/s
                out_speed_list.append(out_speed)
            except:
                None

    if len(in_speed_list):
        avg_in_speed = sum(in_speed_list)/len(in_speed_list)
        max_in_speed = max(in_speed_list)
    
    if len(out_speed_list):
        avg_out_speed = sum(out_speed_list)/len(out_speed_list)
        max_out_speed = max(out_speed_list)

    check_log.append('Max From Device Speed : %f MB/s'%(max_out_speed/1000))
    check_log.append('Avg From Device Speed : %f MB/s'%(avg_out_speed/1000))
    check_log.append('Avg To Device Speed : %f MB/s'%(avg_in_speed/1000))
    check_log.append('Max To Device Speed : %f MB/s'%(max_in_speed/1000))

    print('Max From Device Speed : %f MB/s'%(max_out_speed/1000))
    print('Avg From Device Speed : %f MB/s'%(avg_out_speed/1000))
    print('Avg To Device Speed : %f MB/s'%(avg_in_speed/1000))
    print('Max To Device Speed : %f MB/s'%(max_in_speed/1000))
    print('\n')

def check_data_rate(target,robot=None):
    global check_log
    # https://github.com/IntelRealSense/librealsense/tree/master/tools/data-collect

    usbtop_proc = Popen(usbtop_cmd,stdout=PIPE,shell=True)

    if robot:
        scan_head_thread = Thread(target=scan_head_sequence,args=[robot,])
        scan_head_thread.start()

    cmd='rs-data-collect -c /tmp/d435i_confg.cfg -f /tmp/d435i_log.csv -t %d -m %d'%(target['duration'],target['nframe'])
    out = Popen(cmd, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()    
    usbtop_proc.kill()

    ff=open('/tmp/d435i_log.csv') 
    data=ff.readlines()
    data=data[10:] #drop preamble
    check_frames_collected(data,target)
    check_FPS(data)
    ff.close()

    if robot:
        scan_head_thread.join()
    os.system('sudo pkill usbtop')

def get_head_pos(robot,Print=False):
    tilt_pos = robot.status['head']['head_tilt']['pos']
    pan_pos = robot.status['head']['head_pan']['pos']
    if Print:
        print('Head Tilt: %f | Pan: %f' % (tilt_pos,pan_pos))
    return (pan_pos,tilt_pos)

def save_collected_log(check_log):
    print('---------- COLLECTED LOG ----------')
    check_log_str = ''
    for ll in check_log:
        check_log_str = check_log_str + ll + '\n'
    
    check_log_file = open(log_file_path,"w")
    check_log_file.write(check_log_str)
    print('Collected D435i Check log saved at "'+log_file_path+'"')

def scan_head_sequence(robot):
    """
    Head Pan Tilt Sequence
    """
    global pan_tilt_pos
    robot.head.home()
    time.sleep(1)

    n = 15
    delay = 0.1
    tilt_moves = np.linspace(-1.57,0,n)
    pan_moves = np.linspace(1.57,-3.14,n)

    for j in range(0,3):
        for i in range(n):
            robot.head.move_to('head_tilt',tilt_moves[i])
            robot.head.move_to('head_pan',pan_moves[i])
            time.sleep(delay)
            pan_tilt_pos = get_head_pos(robot)
        time.sleep(0.8)

        for i in range(n):
            robot.head.move_to('head_tilt',np.flip(tilt_moves)[i])
            robot.head.move_to('head_pan',np.flip(pan_moves)[i])
            time.sleep(delay)
            pan_tilt_pos = get_head_pos(robot)
        time.sleep(0.8)
            
        for i in range(n):
            robot.head.move_to('head_tilt',np.flip(tilt_moves)[i])
            robot.head.move_to('head_pan',pan_moves[i])
            time.sleep(delay)
            pan_tilt_pos = get_head_pos(robot)
        time.sleep(0.8)

        for i in range(n):
            robot.head.move_to('head_tilt',tilt_moves[i])
            robot.head.move_to('head_pan',np.flip(pan_moves)[i])
            time.sleep(delay)
            pan_tilt_pos = get_head_pos(robot)
        time.sleep(0.8)

    robot.head.home()

def scan_head_check_rate():
    """
    Check D435i rates with head moving to extremities
    """
    global thread_stop, dmesg_log
    check_install_usbtop()
    get_usb_busID()
    check_usb()
    check_ros()
    robot=stretch_body.robot.Robot()
    robot.startup()

    hdu.exec_process(['sudo', 'dmesg', '-c'], True)
    hdu.exec_process(['sudo', 'modprobe', 'usbmon'], True)
    thread_stop = False
    monitor_dmesg = Thread(target=check_dmesg_thread)
    monitor_dmesg.start()

    conf_type = '---------- HIGH RES CHECK ----------'
    check_log.append('\n'+conf_type + '\n')


    print(conf_type)
    print('Checking high-res data rates. This will take 30s...')
    target=create_config_target_hi_res()
    check_data_rate(target,robot)
    check_throughput('/tmp/usbrate.txt')
    time.sleep(1.5)

    conf_type = '---------- LOW RES CHECK ----------'
    check_log.append('\n'+conf_type + '\n')
    print(conf_type)
    print('Checking low-res data rates. This will take 30s...')
    target=create_config_target_low_res()
    check_data_rate(target,robot)
    check_throughput('/tmp/usbrate.txt')
    time.sleep(1.5)

    thread_stop = True
    monitor_dmesg.join()
    
    check_dmesg(dmesg_log)
    save_collected_log(check_log)

    
    robot.stop()

def check_rate_exec():
    """
    Check D435i rates without head moving
    """
    global thread_stop, dmesg_log
    check_install_usbtop()
    get_usb_busID()
    check_usb()
    check_ros()
    hdu.exec_process(['sudo', 'dmesg', '-c'], True)
    hdu.exec_process(['sudo', 'modprobe', 'usbmon'], True)
    

    conf_type = '---------- HIGH RES CHECK ----------'
    check_log.append('\n'+conf_type + '\n')
    thread_stop = False
    monitor_dmesg = Thread(target=check_dmesg_thread)
    monitor_dmesg.start()

    print(conf_type)
    print('Checking high-res data rates. This will take 30s...')
    target=create_config_target_hi_res()
    check_data_rate(target)
    check_throughput('/tmp/usbrate.txt')

    conf_type = '---------- LOW RES CHECK ----------'
    check_log.append('\n'+conf_type + '\n')
    print(conf_type)
    print('Checking low-res data rates. This will take 30s...')
    target=create_config_target_low_res()
    check_data_rate(target)
    check_throughput('/tmp/usbrate.txt')

    thread_stop = True
    monitor_dmesg.join()

    check_dmesg(dmesg_log)
    save_collected_log(check_log)

def check_install_usbtop():
    """
    Function to be executed at start. Checks for usbtop and if not prompts the user for installation.
    """
    out = Popen("which usbtop", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()
    if len(out):
        return None
    else:
        print('"usbtop" tool is required to be installed for running this test first time.')
        x = raw_input('Enter "y" to proceed with Installation of "usbtop".\n')

        if x=='y' or x=='Y':
            script = 'cd ~/'
            script = script+';git clone https://github.com/aguinet/usbtop.git'
            script = script+';cd usbtop'
            script = script+';sudo apt install libboost-dev libpcap-dev libboost-thread-dev libboost-system-dev cmake'
            script = script+';mkdir _build && cd _build'
            script = script+';cmake -DCMAKE_BUILD_TYPE=Release ..'
            script = script+';make'
            script = script+';sudo make install'
            print('Installing usbtop tool.....')
            os.system(script)
            check = Popen("which usbtop", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read()
            if check:
                print(Fore.GREEN +'[Pass] "usbtop" sucessfully installed'+Style.RESET_ALL+'\n\n')
            else:
                print(Fore.RED + '[Fail] "usbtop" did not install.'+Style.RESET_ALL)
                sys.exit()
        else:
            print('Exiting...')
            sys.exit()

if args.f:
    log_file_path = args.f
    
if args.usb:
    get_usb_busID()
    check_usb()

if args.rate:
    check_rate_exec()

if args.scan_head:
    scan_head_check_rate()


