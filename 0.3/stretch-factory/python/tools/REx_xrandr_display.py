#!/usr/bin/env python3

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

import os
import sys
import yaml
import pathlib
import argparse
from Xlib import display
from Xlib.ext import randr

parser = argparse.ArgumentParser(
    description="Tool to change display resolution/fps/etc.."
)
group = parser.add_mutually_exclusive_group(required=False)
group.add_argument('--set', type=str, help='Set resolution to WIDTHxHEIGHTxFPS. E.g. --set 1920x1080x60.00')
group.add_argument('--set-720p', action='store_true', help='Set resolution to 1280x720xhighest_fps')
group.add_argument('--set-1080p', action='store_true', help='Set resolution to 1920x1080xhighest_fps')
group.add_argument('--revert', action='store_true', help='Revert resolution to whatever it was before --set was called. Does nothing if prev resolution not saved or cleared with reboot.')
group.add_argument('--list', action='store_true', help='List all available resolutions for the display')
group.add_argument('--current', action='store_true', help='Print out the current resolution being used')
args = vars(parser.parse_args())

def issue_xrandr_command(output, resolution):
    if hu.get_display() != ":0":
        print("Error: Cannot change resolution when the DISPLAY env var isn't set to ':0'. Try export DISPLAY=':0'")
        sys.exit(1)
    width, height, fps = resolution.split('x')
    os.system(f"xrandr --output {output} --mode {width}x{height} --rate {fps}")

def find_mode(id, modes):
    for mode in modes:
        if id == mode.id:
            return f"{mode.width}x{mode.height}x{mode.dot_clock / (mode.h_total * mode.v_total):.2f}"

def get_display_info():
    try:
        d = display.Display(':0')
    except:
        print('Error: No display available')
        sys.exit(1)

    screen_count = d.screen_count()
    if screen_count == 0:
        print('Error: No display plugged in')
        sys.exit(1)
    elif screen_count != 1:
        print(f'Error: This tool only supports 1 display. There are {screen_count} plugged in')
        sys.exit(1)

    s = d.screen(0)
    window = s.root
    res = randr.get_screen_resources(window)
    result = []
    for output in res.outputs:
        params = d.xrandr_get_output_info(output, res.config_timestamp)
        if not params.crtc:
            continue
        crtc = d.xrandr_get_crtc_info(params.crtc, res.config_timestamp)
        crtc_resolution = f"{crtc.width}x{crtc.height}"
        crtc_mode = find_mode(crtc.mode, res.modes)
        if crtc_mode == None or crtc_resolution not in crtc_mode:
            print('Error: Unable to find resolution mode for current display')
            sys.exit(1)
        modes = set()
        for mode in params.modes:
            modes.add(find_mode(mode, res.modes))
        result.append({
            'name': params.name,
            'resolution': crtc_mode,
            'available_resolutions': list(modes),
        })

    if len(result) == 0:
        print('Error: No display plugged in')
        sys.exit(1)
    elif len(result) != 1:
        print(f'Error: This tool only supports 1 display. There are {len(result)} plugged in')
        sys.exit(1)

    oresult = result[0]
    oresult['available_resolutions'].sort(key=lambda val: int(val.split('x')[0]))

    return oresult

def save_display_info(info):
    f = pathlib.Path(hu.get_stretch_directory()) / 'log' / 'previous_display_resolution.yaml'
    f.parent.mkdir(exist_ok=True, parents=True)
    if f.is_file():
        print('Warning: previous display resolution already saved. Not overwriting.')
        return
    with open(str(f), 'w') as s:
        yaml.dump(info, s)

def load_display_info():
    f = pathlib.Path(hu.get_stretch_directory()) / 'log' / 'previous_display_resolution.yaml'
    if not f.is_file():
        print('Error: cannot find previous display resolution')
        sys.exit(1)
    with open(str(f), 'r') as s:
        try:
            info = yaml.safe_load(s)
        except:
            print('Error: unable to load previous display resolution')
            sys.exit(1)
    return info

def clear_display_info():
    f = pathlib.Path(hu.get_stretch_directory()) / 'log' / 'previous_display_resolution.yaml'
    f.unlink(missing_ok=True)

if args['current']:
    info = get_display_info()
    print(f"Display Name:       {info['name']}")
    print(f"Display Resolution: {info['resolution']}")
elif args['list']:
    info = get_display_info()
    print("Available Resolutions:")
    for r in info['available_resolutions']:
        print(r)
elif args['set']:
    desired_resolution = args['set']
    dr_parts = desired_resolution.split('x')
    if len(dr_parts) != 3:
        print('Error: Resolution should have 3 parts. E.g. 1920x1080x60.00')
        sys.exit(1)
    dr_parts_empty = [dr_part == '' for dr_part in dr_parts]
    if any(dr_parts_empty):
        print('Error: Resolution malformed. E.g. 1920x1080x60.00')
        sys.exit(1)
    info = get_display_info()
    if desired_resolution not in info['available_resolutions']:
        closest_resolution = '1920x1080x60.00'
        width_match_resolutions = [wmr for wmr in info['available_resolutions'] if dr_parts[0] in wmr]
        closest_resolution = width_match_resolutions[0] if len(width_match_resolutions) > 0 else closest_resolution
        wh_match_resolutions = [whmr for whmr in width_match_resolutions if f"{dr_parts[0]}x{dr_parts[1]}" in whmr]
        closest_resolution = wh_match_resolutions[0] if len(wh_match_resolutions) > 0 else closest_resolution
        whf_match_resolutions = [whfmr for whfmr in wh_match_resolutions if float(dr_parts[2]) == float(whfmr.split('x')[2])]
        closest_resolution = whf_match_resolutions[0] if len(whf_match_resolutions) > 0 else closest_resolution
        print(f'Warning: Desired resolution not available. Picking closest: {closest_resolution}')
        desired_resolution = closest_resolution

    issue_xrandr_command(info['name'], desired_resolution)

    old_info = info
    new_info = get_display_info()
    if new_info['resolution'] != desired_resolution:
        print("Warning: Issued xrandr request, but the resolution doesn't seem to have change")
    save_display_info(old_info)
elif args['set_720p']:
    desired_resolution = '1280x720x60.00'
    info = get_display_info()
    if desired_resolution not in info['available_resolutions']:
        print('Error: 720p resolution not available')
        sys.exit(1)
    issue_xrandr_command(info['name'], desired_resolution)

    old_info = info
    new_info = get_display_info()
    if new_info['resolution'] != desired_resolution:
        print("Warning: Issued xrandr request, but the resolution doesn't seem to have change")
    save_display_info(old_info)
elif args['set_1080p']:
    desired_resolution = '1920x1080x60.00'
    info = get_display_info()
    if desired_resolution not in info['available_resolutions']:
        print('Error: 1080p resolution not available')
        sys.exit(1)
    issue_xrandr_command(info['name'], desired_resolution)

    old_info = info
    new_info = get_display_info()
    if new_info['resolution'] != desired_resolution:
        print("Warning: Issued xrandr request, but the resolution doesn't seem to have change")
    save_display_info(old_info)
elif args['revert']:
    prev_info = load_display_info()
    curr_info = get_display_info()
    if prev_info['name'] != curr_info['name']:
        print(f"Error: display has changed, prev={prev_info['name']}, curr={curr_info['name']}")
        sys.exit(1)

    desired_resolution = prev_info['resolution']
    if desired_resolution not in curr_info['available_resolutions']:
        print(f'Error: previous resolution, {desired_resolution}, not available')
        sys.exit(1)
    issue_xrandr_command(curr_info['name'], desired_resolution)

    new_info = get_display_info()
    if new_info['resolution'] != desired_resolution:
        print("Warning: Issued xrandr request, but the resolution doesn't seem to have change")
    clear_display_info()
else:
    parser.print_help()

