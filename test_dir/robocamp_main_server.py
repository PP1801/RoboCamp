#!/usr/bin/env python3

### name:           robocamp_main_server.py
### description:    ROBOCAMP main server
### date:           20.06.2025.

from pymavlink import mavutil
import tkinter as tk
import sys, argparse
import socket
from multiprocessing import Process, Queue, Value
import time
import math, struct
from datetime import datetime

from tkinter import messagebox

from multiprocessing import Array
import ctypes

from robocamp_drone_functions import *
from NMEA_parser import *

cmd_parser = argparse.ArgumentParser(description='Input parameters')
cmd_parser.add_argument('-s', action='store_true', help='use simulation ip')
#cmd_parser.add_argument('-j', action='store_true', help='use jetson ip')
cmd_parser.add_argument('-m', action='store_true', help='use mobile hotspot host')
cmd_parser.add_argument('-n', action='store_true', help='no drone')
cmd_parser.add_argument('-c', action='store_true', help='clean/compact interface')
cmd_args = cmd_parser.parse_args()

if (cmd_args.s):    # drone from simulator
    jetson_address = '127.0.0.1', 4444       # sim/putanec
else:               # drone from robocamp1 network
    jetson_address = '192.168.0.229', 4444   # Jetson/

if (cmd_args.n):    # No drone on network, solely for testing bracelets
    _wait_for_drone = 0
else:               # Wait until the drone connects to the network
    _wait_for_drone = 1

if (cmd_args.m):    # Using mobile hotspot for Wi-Fi network
    #host = '192.168.43.221'
    host = '192.168.46.173' #Studenti
else:               # Using dedicated robocamp1 Wi-Fi router
    host = '192.168.0.145'

port = 3333 # sim/Putanec

VIRTUAL_SERIAL_PORT = "/dev/ttyROBO0"
LOG_FILE_NAME = "pos_log.txt"

global last_address

def udp_server_handler():
    """UDP server loop."""
    global last_address
    while server_enable.value != -1:        # main loop : while not disabled

        while server_enable.value != 1:     # separate loop to ignore messages

            if server_enable.value == 0:
                try:
                    clear_data, clear_address = server_socket.recvfrom(1024)
                except socket.timeout:
                    continue
            else:
                if server_enable.value == -1:   # complete shutdown
                    return  #sys.exit()
                time.sleep(1)
            continue

        try:                        # while running, try to receive and handle messages
            data, client_address = server_socket.recvfrom(1024)

            do_action = 0

            if (data == b'new'):
                if client_address[0] not in bracelet_dict:
                    print("New bracelet connected: ", client_address[0])
                    bracelet_dict[client_address[0]] = ["", "", None, None]
                    queue.put(bracelet_dict)

            elif (data == b'ping1' and client_address == jetson_address):
                set_aux(10)
                print(f"got PING")

            elif(data == b'search'):
                print("Face search  :", client_address[0])
                if auto_send.value == 1:
                    do_action = 2
                else:
                    do_action = 1

            elif (data == b'track'):
                print("Face tracking:", client_address[0])
                if auto_send.value == 1:
                    do_action = 2
                else:
                    do_action = 1
                set_aux(12)

            elif (data == b'gps'):
                print("GPS location :", client_address[0])
                do_action = 3   # 3 to ask for permission via pop-up, 1 to wait for manual button click
                set_aux(12)

            elif (data == b'off'):
                print("OFF request  :",client_address[0])
                do_action = 2
                set_aux(12)

            elif (data == b's1'):
                print("D-- Successful takeoff")
            elif (data == b'f0'):
                print("D-- Fail to takeoff: Not in GUIDED")
            elif (data == b'f-1'):
                print("D-- Fail to takeoff: Not ARMABLE")
            elif (data == b'f-2'):
                print("D-- Fail to takeoff: Mode ERROR")

            elif check_is_nmea(data):

                if show_nmea.value == 1:
                    print(data)

                is_good_data = False

                # if client_address[0] == "192.168.0.116":
                #     data = b"$GNGGA,091939.75,4547.7618390,N,01558.4372229,E,2,12,0.66,123.009,M,41.375,M,,0000*44"

                try:
                    parsed_data = parse_gngga(data, 1)  # Put 0 for sim use, 1 for real use
                    tmp_lat, tmp_lon, tmp_alt = parsed_data['latitude'], parsed_data['longitude'], parsed_data['altitude']
                    fix_q, num_sat, hdop = parsed_data['fix_quality'], parsed_data['num_satellites'], parsed_data['hdop']
                    is_good_data = True
                except:
                    pass

                if is_good_data:
                    bracelet_ip = client_address[0]

                    if bracelet_ip in nmea_dict:
                        current_step = nmea_dict[bracelet_ip] + 1
                        if current_step >= nmea_n_steps.value:
                            current_step = 0
                    else:
                        current_step = nmea_n_steps.value - 1

                    nmea_dict[bracelet_ip] = current_step

                    if show_pos.value == 1:
                        if bracelet_ip in bracelet_dict:
                            tmp_data = bracelet_dict[bracelet_ip]
                            tmp_data[2] = tmp_lat
                            tmp_data[3] = tmp_lon
                            bracelet_dict[bracelet_ip] = tmp_data
                            queue.put(bracelet_dict)
                        else:
                            bracelet_dict[bracelet_ip] = ["", "", tmp_lat, tmp_lon]
                            queue.put(bracelet_dict)

                        ptype = type_map.get(bracelet_dict[bracelet_ip][0], -1)
                        pstate = bracelet_dict[bracelet_ip][1]
                        if pstate is None or pstate == "":
                            pstate = 0
                        else:
                            pstate = int(pstate)

                        serial_data = f"{bracelet_ip},{ptype},{pstate},{tmp_lat:.7f},{tmp_lon:.7f},12"

                        send_planner(serial_data)   # Refresh marker
                        
                        if current_step == nmea_n_steps.value - 1:      # Only send every n-th message
                            #if forward_nmea.value == 1 and bracelet_ip in bracelet_dict:
                            if forward_nmea.value == 1 and bracelet_ip == active_bracelet.value.decode():
                                if bracelet_dict[bracelet_ip][1] == 1:
                                    _height = follow_height.value
                                    _offset = follow_radius.value
                                    follow_data = f"%{bracelet_ip},{tmp_lat:.7f},{tmp_lon:.7f},{_height:.2f},{_offset:.2f}".encode('utf-8')
                                    server_socket.sendto(follow_data, jetson_address)

                        #if do_log.value == 1 and bracelet_ip in bracelet_dict:
                        if do_log.value == 1 and bracelet_ip == active_bracelet.value.decode():
                            if current_step == nmea_n_steps.value - 1:      # Only write every n-th message
                                timestamp = datetime.now().strftime("%H:%M:%S")
                                pos_log = f"{timestamp},{tmp_lat:.7f},{tmp_lon:.7f},{fix_q},{num_sat},{hdop}"
                                #pos_log = f"{timestamp},{tmp_lat},{tmp_lon}\n"
                                print(pos_log)
                                with open(LOG_FILE_NAME, "a") as file:
                                    file.write(pos_log)
                                    file.write("\n")

                elif (cmd_args.s):   # not is_good_data and we are strictly simulating
                    if show_nmea.value == 1:
                        print("offline dat")    

                    bracelet_ip = client_address[0]

                    if bracelet_ip in nmea_dict:
                        current_step = nmea_dict[bracelet_ip] + 1
                        if current_step >= nmea_n_steps.value:
                            current_step = 0
                    else:
                        current_step = nmea_n_steps.value - 1

                    nmea_dict[bracelet_ip] = current_step

                    if show_pos.value == 1:
                        if bracelet_ip not in bracelet_dict:
                            bracelet_dict[bracelet_ip] = ["", "", None, None]

                        ptype = type_map.get(bracelet_dict[bracelet_ip][0], -1)
                        pstate = bracelet_dict[bracelet_ip][1]
                        if pstate is None or pstate == "":
                            pstate = 0
                        else:
                            pstate = int(pstate)

                        #45.7953420,15.9730817
                        tmp_lat = 45.7952420 + 0.00004*ptype
                        tmp_lon = 15.9730217 + 0.00004*ptype

                        tmp_data = bracelet_dict[bracelet_ip]
                        tmp_data[2] = tmp_lat
                        tmp_data[3] = tmp_lon
                        bracelet_dict[bracelet_ip] = tmp_data
                        queue.put(bracelet_dict)

                        if current_step == nmea_n_steps.value - 1:      # Only send every n-th message
                            #if forward_nmea.value == 1 and bracelet_ip in bracelet_dict:
                            if forward_nmea.value == 1 and bracelet_ip == active_bracelet.value.decode():
                                if bracelet_dict[bracelet_ip][1] == 1:
                                    _height = follow_height.value
                                    _offset = follow_radius.value
                                    follow_data = f"%{bracelet_ip},{tmp_lat:.7f},{tmp_lon:.7f},{_height:.2f},{_offset:.2f}".encode('utf-8')
                                    server_socket.sendto(follow_data, jetson_address)

                        serial_data = f"{bracelet_ip},{ptype},{pstate},{tmp_lat:.7f},{tmp_lon:.7f},12"
                        send_planner(serial_data)   # Refresh marker

                elif show_nmea.value == 1:      # BAD data, not simulation
                    print("BAD GPS data")

            elif check_is_cmd(data):
                parsed_data = parse_cmd(data)
                ack, bracelet_ip = parsed_data['cmd'], parsed_data['bip']
                print(f"got {ack} for : {bracelet_ip}")   # Got confirmation from drone;
                bracelet_addr = bracelet_ip, port        # Send confirmation to bracelet
                server_socket.sendto(ack.encode('utf-8'), bracelet_addr)

                for bracelet_ip in bracelet_dict:
                    tmp_data = bracelet_dict[bracelet_ip]   # full data row

                    if bracelet_ip == active_bracelet.value.decode():
                        tmp_data[1] = 1     # Enable state for targeted/confirmed bracelet

                        if ack == "off1" or ack == "off3":
                            tmp_data[0] = "off"
                            tmp_data[1] = 2 # confirmed off
                        send_planner(f"{bracelet_ip},3")  # Modify marker

                    else:
                        tmp_data[1] = 0     # Disable all non-targeted bracelets

                    bracelet_dict[bracelet_ip] = tmp_data   # update data row

                queue.put(bracelet_dict)            #Update window state
                #print(bracelet_dict)

                if ack == "gps1":
                    set_aux(13)

            elif check_is_ina(data):
                print(f"ina data: {data} from {client_address[0]}")

            elif print_all.value == 1:   # Other type of data
                print(f"data: {data} from {client_address[0]}")

            do_action_function(do_action, data, client_address)
            
            last_address = client_address

            do_aux_action_function()

        except socket.timeout:      # no new messages, do other functions if needed

            do_aux_action_function()

            continue

    print("U-- server ended")

def do_action_function(_do_action, _data, _client_address=None, _bracelet_ip=None):
    """Do action depending on the input do_action int >=
    \n (1) Update bracelet window state,
    \n (2) Send bracelet data to jetson"""
    if _do_action >= 1:

        if _bracelet_ip != None:
            bracelet_ip = _bracelet_ip
        else:
            bracelet_ip = _client_address[0]

        if _do_action == 2:     # Automatically send request to jetson
            bracelet_data = f"#{_data.decode('utf-8')},{bracelet_ip}".encode('utf-8')
            server_socket.sendto(bracelet_data, jetson_address)

        elif _do_action == 3:   # Send request to jetson after confirmation
            set_aux(12)

        #Update window state
        if bracelet_ip in bracelet_dict:
            tmp_data = bracelet_dict[bracelet_ip]
            tmp_data[0] = _data.decode('utf-8')
            tmp_data[1] = ""
            bracelet_dict[bracelet_ip] = tmp_data
            queue.put(bracelet_dict)
        else:
            bracelet_dict[bracelet_ip] = [_data.decode('utf-8'), "", None, None]
            queue.put(bracelet_dict)

def do_aux_action_function():
    """Do auxiliary action depending on the aux_func.value:
    \n (1) Toggle INA measurement,
    \n (2) Clear bracelet window,
    \n (3) Turn off selected bracelet"""
    global last_address

    if aux_func.value == 1:     # Toggle INA260 measurements
        try:
            print(f"Sent ina to {last_address[0]}")
            server_socket.sendto(b'ina', last_address)
        except Exception as e:
            print(f"No ina sent: {e}")
        set_aux(0)

    elif aux_func.value == 2:   # Clear bracelet
        bracelet_dict.clear()
        queue.put(bracelet_dict)

        send_planner(f"off,0")  # Clear all markers
        set_aux(0)

    elif aux_func.value == 3:   # Turn bracelets off

        for bracelet_ip in bracelet_dict:
            if bracelet_ip == active_bracelet.value.decode():
                #do_action_function(2, "off", _bracelet_ip=bracelet_ip)
                do_action_function(2, b'off', (bracelet_ip, port))   # update dict and notify drone
            else:
                do_action_function(1, b'off', (bracelet_ip, port))   # update dict

            server_socket.sendto(b'off2', (bracelet_ip, port))   # Send brute off to bracelet

        set_aux(13) # destroy GPS popup if it exists

def send_planner(_serial_data):
    """Send marker data to Mission Planner via Virtual Serial Port."""
    try:
        with open(VIRTUAL_SERIAL_PORT, 'wb') as port:
            _serial_data = _serial_data.encode('utf-8')
            port.write(_serial_data)
            port.write(b'\n')
    except Exception as e:
        print(e)

def wait_for_drone(drone_address, pings=1):
    """Send ping to drone address, block server and wait for ping1 response."""
    n_pings = 0

    print(f"U-- Waiting for drone on: {drone_address}")
    while (n_pings < pings):

        server_socket.sendto(b'ping', drone_address)
        data, client_address = server_socket.recvfrom(1024)

        if data == b'ping1':
            n_pings = n_pings + 1
            print(f"Received {n_pings}. ping1!")
    return 1

def start_server(_host, _port, _wait_drone=1):
    """Start UDP server and wait for drone ping response."""

    server_socket.bind((_host, _port))
    print(f'U-- UDP server listening on {_host}:{_port}')

    if _wait_drone == 1:
        if wait_for_drone(jetson_address, 1):
            print(f"U-- Successfully connected: {jetson_address}")
    else:
        print("Starting without drone")

    set_aux(10)

    server_socket.settimeout(1)

    time.sleep(0.1)
    server_enable.value = 1

def keyboard_keydown(e):
    """Keyboard command handle."""
    if   e.keysym == 'space':
        server_socket.sendto(b'stop', jetson_address)
        print(">> Stop")
    elif e.keysym == 'Escape':
        print("K-- Keyboard closed")
        sys.exit()

def on_click_activate(input):
    """Activate bracelet button event"""
    bracelet_ip, state = input[0], input[1]
    with active_bracelet.get_lock():
        if active_bracelet.value.decode() != bracelet_ip:
            active_bracelet.value = bracelet_ip.encode('utf-8')
            print(f"New active bracelet = {active_bracelet.value.decode()}")
        else: 
            print(f"retry...")
    print(f"Allowing, try: {state} for {bracelet_ip}")
    bracelet_data = f"#{state},{bracelet_ip}".encode('utf-8')
    server_socket.sendto(bracelet_data, jetson_address)

def on_button_click(btn_name):
    """Button click event handler, if-list of functions by button name."""
    if btn_name == "server_toggle":   
        """Start or stop the server."""
        global btn_server_toggle
        if server_enable.value == 1:
            with server_enable.get_lock():
                server_enable.value = 0
            print("Server stopped!")
            btn_server_toggle.config(text="START", bg="red")
        else:
            with server_enable.get_lock():
                server_enable.value = 1
            print("Server started!")
            btn_server_toggle.config(text="STOP", bg="lightgray") 
  
    elif server_enable.value == 1:  # Do button function if we are connected
        if   btn_name == "btn_rtl":
            server_socket.sendto(b'rtl', jetson_address)
            print(">> Set RTL")
        elif btn_name == "btn_land":
            server_socket.sendto(b'land', jetson_address)
            print(">> Set LAND")
        elif btn_name == "btn_loiter":
            server_socket.sendto(b'loiter', jetson_address)
            print(">> Set LOITER")
        elif btn_name == "btn_guided":
            server_socket.sendto(b'guided', jetson_address)
            print(">> Set GUIDED")
        elif btn_name == "btn_arm":
            server_socket.sendto(b'arm', jetson_address)
            print(">> Try ARMING")
        elif btn_name == "btn_takeoff":
            server_socket.sendto(b'takeoff', jetson_address)
            print("Try to takeoff...")
        elif btn_name == "btn_ping":
            server_socket.sendto(b'ping', jetson_address)
            print(">> Sent PING")
        elif btn_name == "btn_distp":
            server_socket.sendto(b'lin+', jetson_address)
            print(">> DIST+")
        elif btn_name == "btn_distm":
            server_socket.sendto(b'lin-', jetson_address)
            print(">> DIST-")
        elif btn_name == "btn_repos":
            server_socket.sendto(b'posr', jetson_address)
            print(">> REPOS")
        elif btn_name == "btn_point1":
            server_socket.sendto(b'p1', jetson_address)
            print("goto point 1")
        elif btn_name == "btn_point2":
            server_socket.sendto(b'p2', jetson_address)
            print("goto point 2")
        elif btn_name == "btn_point3":
            server_socket.sendto(b'p3', jetson_address)
            print("goto point 3")
        elif btn_name == "btn_point4":
            server_socket.sendto(b'p4', jetson_address)
            print("goto point 4")
        elif btn_name == "btn_ina":
            set_aux(1)
        elif btn_name == "list_clear":
            set_aux(2)
            print("Clear bracelet list")
        elif btn_name == "btn_boff":
            set_aux(3)
        elif btn_name == "btn_fence":
            """Toggle fence for keeping distance"""
            global btn_fence
            with keep_distance.get_lock():
                keep_distance.value = 1 - keep_distance.value
            msg = f"fence{keep_distance.value}"
            server_socket.sendto(msg.encode('utf-8'), jetson_address)
            print(f"Toggle Fence to {keep_distance.value}")

            set_aux(11)

            if keep_distance.value == 1:
                btn_fence.config(bg="green", fg="white")
            else:
                btn_fence.config(bg="lightgray", fg="black")

        elif btn_name == "btn_fnmea":
            """Toggle NMEA forwarding to drone"""
            global btn_fnmea
            with forward_nmea.get_lock():
                forward_nmea.value = 1 - forward_nmea.value
            print(f"Toggle NMEA forwarding to {forward_nmea.value}")
            if forward_nmea.value == 1:
                btn_fnmea.config(bg="green", fg="white")
            else:
                btn_fnmea.config(bg="lightgray", fg="black")

        elif btn_name == "btn_snmea":
            """Toggle NMEA print to console"""
            global btn_snmea
            with show_nmea.get_lock():
                show_nmea.value = 1 - show_nmea.value
            print(f"Toggle NMEA console display to {show_nmea.value}")
            if show_nmea.value == 1:
                btn_snmea.config(bg="green", fg="white")
            else:
                btn_snmea.config(bg="lightgray", fg="black")

        elif btn_name == "btn_showpos":
            """Toggle show position"""
            global btn_showpos
            with show_pos.get_lock():
                show_pos.value = 1 - show_pos.value
            serial_data = f"poss{show_pos.value}"
            server_socket.sendto(serial_data.encode('utf-8'), jetson_address)
            print("Toggle show position to: ", show_pos.value)
            if show_pos.value == 1:
                btn_showpos.config(bg="green", fg="white")
            else:
                btn_showpos.config(bg="lightgray", fg="black")

        elif btn_name == "btn_plog":
            """Start or stop location logging into pos_log.txt"""
            global btn_plog
            with do_log.get_lock():
                do_log.value = 1 - do_log.value
            if do_log.value == 1:
                print(f"Start logging to file {LOG_FILE_NAME}")
                btn_plog.config(bg="green", fg="white")
            else:
                print(f"Stop  logging to file {LOG_FILE_NAME}")
                btn_plog.config(bg="lightgray", fg="black")
        
        elif btn_name == "btn_disp":
            """Toggle printing of other incoming messages in console"""
            with print_all.get_lock():
                print_all.value = 1 - print_all.value
            print("Toggle message print to: ", print_all.value)

        elif btn_name == "btn_autot":
            """Toggle automatic switch between search-track modes"""
            with auto_try.get_lock():
                auto_try.value = 1 - auto_try.value
            print("Toggle autotry to: ", auto_try.value)
            if auto_try.value == 1:
                btn_autot.config(bg="green", fg="white")
            else:
                btn_autot.config(bg="lightgray", fg="black")

            msg = f"autot{auto_try.value}"
            server_socket.sendto(msg.encode('utf-8'), jetson_address)

        elif btn_name == "btn_autos":
            """Toggle automatic switch between search-track modes"""
            with auto_send.get_lock():
                auto_send.value = 1 - auto_send.value
            print("Toggle auto send to: ", auto_send.value)
            if auto_send.value == 1:
                btn_autos.config(bg="green", fg="white")
            else:
                btn_autos.config(bg="lightgray", fg="black")

    elif btn_name == "btn_ping":        # Send ping even if we are disconnected
        server_socket.sendto(b'ping', jetson_address)
        print(">> Sent PING")
    else:
        print(">> Not Connected!")   

if (cmd_args.c):    # compact/cleaner interface
    main_button_matrix = [
        ["btn_land",    "btn_rtl",    "btn_loiter", "btn_ping"],
        ["btn_takeoff", "btn_arm", "btn_guided",    "btn_more1"],
    ]
    """List with main functions button matrix."""

    more_button_matrix = [
        [
            ["btn_fnmea", "fol_height", "btn_fence", "fol_radius"],
            ["btn_boff",  "btn_autos",  "btn_autot", "list_clear"],   
        ],
    ]   
    """List with pages of additional function button matrices."""
else:               # normal dev interface
    main_button_matrix = [
        ["btn_more1", "btn_rtl",    "btn_guided",   "server_toggle"],
        ["btn_more2", "btn_land",   "btn_arm",      "btn_ping"],
        ["btn_more3", "btn_loiter", "btn_takeoff"],
    ]
    """List with main functions button matrix."""

    more_button_matrix = [
        [
            ["btn_boff",    "btn_snmea", "btn_fnmea",  "nmea_steps"],
            ["btn_repos",  "fol_height", "btn_fence",  "fol_radius"],   
            ["btn_autos", "btn_autot",],
        ],
        [
            ["btn_ina", "list_clear", "btn_plog"],
            ["btn_disp", "E", "btn_snmea"],
            ["G", "H", "I"],
        ],
        [   
            ["btn_showpos", "btn_distp", "btn_point4", "btn_point3"],
            ["btn_plog",    "btn_distm", "btn_point1", "btn_point2"],
            ["list_clear",],
        ]
    ]
    """List with pages of additional function button matrices."""

button_info = {
    "btn_arm"   :   ["ARM"  ,   ""],
    "btn_rtl"   :   ["RTL"  ,   ""],
    "btn_land"  :   ["LAND" ,   ""],
    "btn_loiter":   ["LOITER",  ""],
    "btn_guided":   ["GUIDED",  ""],
    "btn_takeoff":  ["TAKEOFF", "Start takeoff procedure"],
    "btn_ping"  :   ["PING" ,   "Send ping to Jetson"],
    "btn_fence" :   ["FENCE",   "Toggle circular fence around braclet location"],
    "btn_fnmea" :   ["(F)NMEA", "Toggle NMEA message forward to drone"],
    "btn_snmea" :   ["(S)NMEA", "Toggle NMEA message print in console"],
    "btn_distp" :   ["DIST+",   "Increase (+) distance between points by 100"],
    "btn_distm" :   ["DIST-",   "Decrease (-) distance between points by 100"],
    "btn_showpos":  ["DISP P",  "Show bracelet's position"],
    "btn_repos" :   ["REPOS",   "Reposition drone around the last point"],
    "btn_ina"   :   ["INA260",  "Check INA260 current, voltage and power"],
    "list_clear":   ["CLEAR",   "Clear bracelet list"],
    "btn_more3" :   ["more3",   "Bracelet and position functions"],
    "btn_more2" :   ["more2",   ""],
    "btn_more1" :   ["more1",   ""],
    "btn_point4":   ["POINT 4", ""],
    "btn_point3":   ["POINT 3", ""],
    "btn_point2":   ["POINT 2", ""],
    "btn_point1":   ["POINT 1", ""],
    "btn_boff"  :   ["b OFF",   "Change last bracelet state to off"],
    "server_toggle":["STOP",    "Start/Stop handling incoming messages, you can still send"],
    "nmea_steps":   ["",        "Update/Forward NMEA message every n seconds"],
    "btn_plog"  :   ["LOG N",   "Start logging bracelet position into pos_log.txt"],
    "btn_disp"  :   ["DISP M",  "Toggle print of other incoming messages"],
    "btn_autot" :   ["Auto T",  "Toggle auto-retry for search-track modes"],
    "btn_autos" :   ["Auto S",  "Toggle auto send from bracelet to drone (use with caution)"],
    "fol_radius":   ["",        "Set offset radius to this many [m] away from bracelet"],
    "fol_height":   ["",        "Set drone follow height to this many [m] AGL (above terrain)"],
}
"""Button text [0] and help text pop-up [1] by button key name."""

color_map = {
    "gps"   : "green",
    "search": "blue",
    "track" : "yellow",
    "off"   : "white",
    "disconnect": "red",
    "blue"  : "white",
    "green" : "white",
}
"""Returns button background color depending on the bracelet state, or
returns button text color depending on the button background color."""

type_map = {
    "off"   : 0,
    "gps"   : 1,
    "search": 2,
    "track" : 3,
}

def set_aux(_int_val):
    with aux_func.get_lock():
        aux_func.value = _int_val

global more_buttons_visible
more_buttons_visible = 0



def run_tkinter(queue):
    """Tkinter interface initialization and loop."""

    def on_enter(event, help_button):
        """Update the help label with text for the hovered button."""
        help_text = button_info.get(help_button, ["",""])
        help_label.config(text=help_text[1])

    def on_leave(event):
        """Clear the help label when the mouse leaves the button."""
        help_label.config(text="")

    def on_leave_nmea(event):
        """Update nmea_n_steps ammount after leaving input field."""
        new_n = entry_nmea_steps.get()
        if new_n.isdigit() and int(new_n) in range(1,20):
            if int(new_n) != nmea_n_steps.value:
                with nmea_n_steps.get_lock():
                    nmea_n_steps.value = int(new_n)
                print(f"NMEA steps:{nmea_n_steps.value}")
        else:
            fix_text = tk.StringVar(value=nmea_n_steps.value)
            entry_nmea_steps.config(textvariable=fix_text)
        help_label.config(text="")

    def on_leave_radius(evemt):
        """Update offset radius ammount after leaving input field."""
        input_t = entry_fol_radius.get()
        try:
            new_f = float(input_t)
            if new_f >= float(follow_radius_limit[0]) and new_f <= float(follow_radius_limit[1]):
                with follow_radius.get_lock():
                    follow_radius.value = round(new_f, 1)   # round to 2 decimal places
                print(f"Offset radius set to {follow_radius.value:.1f} [m]")
            else:
                print(f"Offset radius out of bonds: {follow_radius_limit[0]} to {follow_radius_limit[1]}")
                fix_text = tk.StringVar(value=round(follow_radius.value,1))
                entry_fol_radius.config(textvariable=fix_text)
        except Exception as e:
            print(e)
            print(f"Non float input, keeping: {follow_radius.value:.1f}")

            fix_text = tk.StringVar(value=round(follow_radius.value,1))
            entry_fol_radius.config(textvariable=fix_text)
        help_label.config(text="")
            

    def on_leave_height(event):
        """Update drone follow height after leaving input field"""
        input_t = entry_fol_height.get()
        try:
            new_f = float(input_t)
            if new_f >= float(follow_height_limit[0]) and new_f <= float(follow_height_limit[1]):
                with follow_height.get_lock():
                    follow_height.value = round(new_f, 1)   # round to 2 decimal places
                print(f"Follow height set to {follow_height.value:.1f} [m]")
            else:
                print(f"Follow height out of bonds: {follow_height_limit[0]} to {follow_height_limit[1]}")
                fix_text = tk.StringVar(value=round(follow_height.value,1))
                entry_fol_height.config(textvariable=fix_text)
        except Exception as e:
            print(e)
            print(f"Non float input, keeping: {follow_height.value:.1f}")

            fix_text = tk.StringVar(value=round(follow_height.value,1))
            entry_fol_height.config(textvariable=fix_text)
        help_label.config(text="")

    def on_leave_plog(event):
        """Update position log filename after leaving input field."""
        #pseudo
        #new_n = entry_plog_name.get()
        #if new_n exists:
        #    a

    def destroy_popup(target_title="GPS takeoff"):
        for widget in root.winfo_children():
            if isinstance(widget, tk.Toplevel) and widget.title() == target_title:
                widget.destroy()

    def deny_bracelet(_bracelet_ip):
        # server_socket.sendto(b'off2', (_bracelet_ip, port))
        # do_action_function(1, "off".encode('utf-8'), _bracelet_ip=_bracelet_ip)

        set_aux(3) # brute off2 to all bracelets
        do_aux_action_function()

        # print(bracelet_dict)
        # tmp_data = bracelet_dict[_bracelet_ip]   # full data row
        # tmp_data[0] = "off"
        # tmp_data[1] = 2 # confirmed off
        # bracelet_dict[_bracelet_ip] = tmp_data   # update data row
        # queue.put(bracelet_dict)            #Update window state

        set_aux(12) # reopen popup

    # def deny_all():
    #     set_aux(3) # brute off2 to all bracelets
    #     do_aux_action_function()

    def map_button(_button_name, _button):
        """Link new button to global named object we use to modify the button from other places
        \n Button objects are declared within run_tkinter"""
        global btn_plog, btn_snmea, btn_fnmea, btn_showpos, btn_fence, btn_autot, btn_autos

        if _button_name == "btn_plog":
            btn_plog = _button      
        elif _button_name == "btn_snmea":
            btn_snmea = _button
        elif _button_name == "btn_fnmea":
            btn_fnmea = _button
        elif _button_name == "btn_showpos":
            if show_pos.value == 1: _button.config(bg="green", fg="white")
            btn_showpos = _button
        elif _button_name == "btn_fence":
            btn_fence = _button
        elif _button_name == "btn_autot":
            btn_autot = _button
        elif _button_name == "btn_autos":
            btn_autos = _button

    def toggle_matrix(page):
        """Show or hide the additional button matrix."""
        global more_buttons_visible

        if page > more_button_matrix.__len__():
            return

        for page_number, pages in enumerate(more_buttons):  # Hide all additional buttons
            for row_index, row in enumerate(pages):
                for col_index, button_name in enumerate(row):
                    button_name.grid_remove()
        btn_more[more_buttons_visible-1].config(text=f"more{more_buttons_visible}", bg="lightgray")

        if more_buttons_visible != page:

            for index, wpage in enumerate(more_buttons):    # Update button text on all btn_more{n}
                if index == page-1:                         # Show selected page
                    btn_more[index].config(text="hide", bg="gray")
                    for row_index, row in enumerate(wpage):
                        for col_index, button_name in enumerate(row):
                            button_name.grid(row=6+row_index, column=col_index, padx=10, pady=10, sticky="ew")
                else:
                    btn_more[index].config(text=f"more{index+1}", bg="lightgray")
            more_buttons_visible = page

        else:                                               # Hide selected page
            more_buttons_visible = 0

    def initialize_main_buttons():
        """Initialize main button matrix objects and display them."""
        global btn_server_toggle, btn_ping
        for row_index, row in enumerate(main_button_matrix):
            for col_index, button_name in enumerate(row):

                display_text = button_info.get(button_name, [button_name, ""])

                if button_name.startswith("btn_more"):
                    page_index = int(button_name[-1])
                    new_button = tk.Button(root, text=f"more{page_index}", width=8, bg="lightgray",
                                        command=lambda input=page_index: toggle_matrix(input))
                    new_button.grid(row=row_index+1, column=col_index, padx=10, pady=10, sticky="ew")
                    new_button.bind("<Enter>", lambda event, text=button_name: on_enter(event, text))
                    new_button.bind("<Leave>", on_leave)
                    btn_more.append(new_button)

                else:
                    button = tk.Button(root, text=display_text[0], width=8,
                        command=lambda name=button_name: on_button_click(name))
                    button.grid(row=row_index+1, column=col_index, padx=10, pady=10, sticky="ew")
                    button.bind("<Enter>", lambda event, text=button_name: on_enter(event, text))
                    button.bind("<Leave>", on_leave)
                    # List of button objects that we can modify from other places (change color...)
                    if button_name == "server_toggle":
                        btn_server_toggle = button
                    elif button_name == "btn_ping":
                        btn_ping = button
                        if aux_func.value == 10:
                            btn_ping.config(bg = "green", fg="white")
                            set_aux(0)
                        else:
                            btn_ping.config(bg="red")

    def initialize_more_buttons():
        """Initialize all additional button objects without displaying them."""
        for page in more_button_matrix:
            button_col = []
            for row_index, row in enumerate(page):
                button_row = []
                for col_index, button_name in enumerate(row):
                    if button_name == "nmea_steps":
                        button = entry_nmea_steps
                        button.bind("<Enter>", lambda event, text=button_name: on_enter(event, text))
                        button.bind("<Leave>", on_leave_nmea)  
                    elif button_name == "fol_radius":
                        button = entry_fol_radius
                        button.bind("<Enter>", lambda event, text=button_name: on_enter(event, text))
                        button.bind("<Leave>", on_leave_radius)       
                    elif button_name == "fol_height":
                        button = entry_fol_height
                        button.bind("<Enter>", lambda event, text=button_name: on_enter(event, text))
                        button.bind("<Leave>", on_leave_height)                                                                                      
                    else:
                        display_text = button_info.get(button_name, [button_name, ""])
                        button = tk.Button(root, text=display_text[0], width=8,
                                        command=lambda name=button_name: on_button_click(name))
                        button.bind("<Enter>", lambda event, text=button_name: on_enter(event, text))
                        button.bind("<Leave>", on_leave)
                        map_button(button_name, button)
                    button_row.append(button)
                button_col.append(button_row)
            more_buttons.append(button_col)

    def refresh_bracelet_window():
        """Refresh the bracelet window with new bracelet data."""
        global saved_dict
        while not queue.empty():
            updated_dict = queue.get()

            for index, (key, value) in enumerate(updated_dict.items()):
                if key not in saved_dict:   # New bracelet
                    label_row = []

                    label = tk.Label(bracelet_window, text=key, font=("Arial", 12))
                    label.grid(row=index, column=0, padx=10, pady=10, sticky="ew")
                    label_row.append(label)

                    color = color_map.get(value[0], "lightgray")
                    label = tk.Label(bracelet_window, text=value[0], font=("Arial", 12), width=8,
                                    bg=color, fg=color_map.get(color, "black"))
                    label.grid(row=index, column=1, padx=10, pady=10, sticky="ew")
                    label_row.append(label)

                    button = tk.Button(bracelet_window, text=value[1],font=("Arial", 12), width=2,
                                    command=lambda inputs=(key, value[0]): on_click_activate(inputs))
                    button.grid(row=index, column=2, padx=10, pady=10, sticky="ew")
                    label_row.append(button)

                    if value[2] is None:    f_text = value[2], value[3]
                    else:       f_text = f"{value[2]:.7f}", f"{value[3]:.7f}"

                    label = tk.Label(bracelet_window, text=f_text[0], font=("Arial", 12))
                    label.grid(row=index, column=3, padx=10, pady=10, sticky="ew")
                    label_row.append(label)

                    label = tk.Label(bracelet_window, text=f_text[1], font=("Arial", 12))
                    label.grid(row=index, column=4, padx=10, pady=10, sticky="ew")
                    label_row.append(label)

                    saved_dict_reference[key] = label_row

                elif saved_dict[key] != value:  # Update bracelet state

                    color = color_map.get(value[0], "lightgray")
                    saved_dict_reference[key][1].config(text=value[0], bg=color,
                                                        fg=color_map.get(color, "black"))

                    new_button = tk.Button(bracelet_window, text=value[1],font=("Arial", 12), width=2,
                                command=lambda inputs=(key, value[0]): on_click_activate(inputs))
                    new_button.grid(row=index, column=2, padx=10, pady=10, sticky="ew")
                    saved_dict_reference[key][2].destroy()
                    saved_dict_reference[key][2] = new_button

                    if value[2] is None:    f_text = value[2], value[3]
                    else:       f_text = f"{value[2]:.7f}", f"{value[3]:.7f}"

                    saved_dict_reference[key][3].config(text=f_text[0])
                    saved_dict_reference[key][4].config(text=f_text[1])

            for key in list(saved_dict.keys()): # Remove labels for forgotten bracelets
                if key not in updated_dict:
                    for widget in saved_dict_reference[key]:
                        widget.destroy()
                    del saved_dict_reference[key]

            saved_dict = updated_dict.copy()    # Save current state for later comparison

            if aux_func.value == 10:
                btn_ping.config(bg="green", fg="white")  #global btn_ping
                
                set_aux(0)

            elif aux_func.value == 11:
                if keep_distance.value == 1:
                    entry_fol_radius.config(bg="green", fg="white")
                else:
                    entry_fol_radius.config(bg="lightgray", fg="black")

                set_aux(0)

            elif aux_func.value == 12:
                #if updated_dict is not None:
                #print(updated_dict)
                destroy_popup()
                popup_window = tk.Toplevel(root)
                popup_window.title("GPS takeoff")
                _label = tk.Label(popup_window, text="Approve this GPS?: ", font=("Arial", 12), width=14)
                _label.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
                # _no_to_all = tk.Button(popup_window, text="NO(all)", font=("Arial", 12), width=4,
                #                     command=lambda : deny_all())
                # _no_to_all.grid(row=0, column=1, padx=10, pady=10, sticky="ew")
                _i = 0
                for index, (key, value) in enumerate(updated_dict.items()):

                    if value[0] == "gps":
                        _yes_button = tk.Button(popup_window, text=key, font=("Arial", 12), width=12, bg="green",
                                    command=lambda inputs=(key, value[0]): on_click_activate(inputs))
                        #_no_button = tk.Button(popup_window, text="NO", font=("Arial", 12), width=2,
                        #            command=lambda inputs=(key): deny_bracelet(inputs))
                        _yes_button.grid(row=_i+1, column=0, padx=10, pady=10, sticky="ew")
                        #_no_button.grid(row=_i+1, column=1, padx=10, pady=10, sticky="ew")
                        _i += 1

                if _i == 0:
                    destroy_popup()
                set_aux(0)
            
            elif aux_func.value == 13:
                destroy_popup()
                set_aux(0)

        root.after(500, refresh_bracelet_window)


    root = tk.Tk()
    root.title("RoboCamp Interface extension")

    help_label = tk.Label(root, text="", fg="blue",)
    help_label.grid(row=0, column=0, columnspan=4, pady=10, padx=10)

    btn_more = []
    more_buttons = []
    global btn_server_toggle, btn_plog, btn_snmea, btn_fnmea, btn_showpos, btn_fence, btn_ping, btn_autot, btn_autos

    text_steps = tk.StringVar(value=nmea_n_steps.value)
    text_offset = tk.StringVar(value=follow_radius.value)
    text_height = tk.StringVar(value=follow_height.value)
    entry_nmea_steps = tk.Entry(root, textvariable=text_steps, width=8, bg="lightgray", )
    entry_fol_radius = tk.Entry(root, textvariable=text_offset, width=8, bg="lightgray", )
    entry_fol_height = tk.Entry(root, textvariable=text_height, width=8, bg="lightgray", )

    initialize_main_buttons()
    initialize_more_buttons()

    root.bind("<KeyPress>", lambda e: keyboard_keydown(e))

    bracelet_window = tk.Toplevel(root)
    bracelet_window.title("Bracelet list")

    saved_dict_reference = {}
    global saved_dict
    saved_dict = {}

    refresh_bracelet_window()

    toggle_matrix(1)

    root.mainloop()

if __name__ == '__main__':

    print("RoboCamp initializing...")

    ### Synchronized shared objects
    server_enable = Value('i', 0)   # (int)  server enable
    forward_nmea = Value('i', 0)    # (bool) Toggle nmea forwarding to drone
    show_nmea = Value('i', 0)       # (bool) Toggle nmea print in console
    keep_distance = Value('i', 0)   # (bool) Toggle fence

    show_pos = Value('i', 1)    # (bool) Toggle target lat/long print in console
    aux_func = Value('i', 0)    # (int) auxillary function enable 
    do_log = Value('i', 0)      # (bool) Toggle logging position into .txt file
    print_all = Value('i', 1)   # (bool) Toggle print of other messages into console
    auto_try = Value('i', 0)    # (bool) Toggle auto switch between search and follow modes (neuron-n)
    auto_send = Value('i', 0)    # (bool) Toggle auto send from bracelet to jetson without manual permission

    follow_radius = Value('f', 1.5)     # (float) [m] default follow offset between drone and bracelet
    follow_height = Value('f', 10.0)    # (float) [m] default flying height (AGL/terrain) for drone 
    follow_radius_limit = [1.5, 5.0]
    follow_height_limit = [3.0, 20.0]

    nmea_dict = {}              # Holds current step for each bracelet IP
    nmea_n_steps = Value('i', 1)# (int) Steps for NMEA display frequency

    active_bracelet = Array(ctypes.c_char, 15)   # IP of targeted bracelet

    bracelet_dict = {}
    queue = Queue()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # root = tk.Tk()
    # root.title("RoboCamp Interface extension")

    tkinter_process = Process(target=run_tkinter, args=(queue,))
    server_process = Process(target=udp_server_handler, args=())#(host, port,))

    tkinter_process.start()     # run_tkinter()

    start_server(host, port, _wait_for_drone)

    server_process.start()

    tkinter_process.join()      # Wait for the exit signal from interface extension
    
    server_enable.value = -1    # Set the enable flag to terminate the server
    server_socket.close()
    
    server_process.join()       # Wait for the server to terminate

    print("RoboCamp terminated")