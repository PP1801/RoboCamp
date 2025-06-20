#!/usr/bin/env python3

from pymavlink import mavutil
import tkinter as tk
import sys
import socket
from multiprocessing import Process, Value
import time
import math

from robocamp_drone_functions import *
from NMEA_parser import *

#def udp_server_handler(host='192.168.0.229', port=3333): # Jetson
def udp_server_handler(host='192.168.0.145', port=3333):# sim/Putanec
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # za TCP

    server_socket.bind((host, port))
    print(f'U-- UDP server listening on {host}:{port}')

    while True:
        data, client_adress = server_socket.recvfrom(1024)

        #print(data)

        if current_point.value == -1:
            print("U-- server ended")
            sys.exit()

        if check_is_nmea(data):
            #print(data)
            parsed_data = parse_gngga(data, 0)
            tmp_lat, tmp_lon = parsed_data['latitude'], parsed_data['longitude']
            
            if manual_target.value == 1:
                bt_lat.value, bt_lon.value = tmp_lat, tmp_lon
                manual_target.value = 2
                print(f"Try following-> lat: {bt_lat.value:.7f}\tlon: {bt_lon.value:.7f}")
            elif show_pos.value == 1:
                print(f"lat: {tmp_lat:.7f}\tlon: {tmp_lon:.7f}")

            print("Parsed Data:")
            for key, value in parsed_data.items():
                print(f"{key}: {value}") 

        elif(data == b'search\n'):
            print("Face search request: ", client_adress)
            server_socket.sendto(b'search1', client_adress)
            current_point.value = 1

        elif (data == b'track\n'):
            print("Face tracking request: ", client_adress)
            server_socket.sendto(b'track1', client_adress)
            current_point.value = 2

        elif (data == b'gps\n'):
            print("GPS location request: ", client_adress)
            server_socket.sendto(b'gps1', client_adress)
            current_point.value = 3

        elif (data == b'off\n'):
            print("OFF request: ",client_adress)
            current_point.value = 0

def keyboard_keydown(e):

    if   e.keysym == 'a':
        print(">> Try ARMING")
        arm(the_connection)

    elif e.keysym == 'g':
        print(">> Set GUIDED")
        mode_send(the_connection, 4)
    elif e.keysym == 'l':
        print(">> Set LAND")
        mode_send(the_connection, 9)
    elif e.keysym == 'r':
        print(">> Set RTL")
        set_return(the_connection)
    # elif e.keysym == 's':
    #     print(">> Set STABILIZE")
    #     mode_send(the_connection, 0)

    elif e.keysym == 'y':
        manual_target.value = 1

    elif e.keysym == '1' or e.keysym == 'KP_1':
        print("point 1")
        current_point.value = 1

    elif e.keysym == '2' or e.keysym == 'KP_2':
        print("point 2")
        current_point.value = 2

    elif e.keysym == '3' or e.keysym == 'KP_3':
        print("point 3")
        current_point.value = 3

    elif e.keysym == '4' or e.keysym == 'KP_4':
        print("point 4")
        current_point.value = 4

    elif e.keysym == 'c':
        print("K-- Keyboard closed")
        current_point.value = -1
        sys.exit()

    elif e.keycode == 86:   # keypad +
        #with point_offset.get_lock():
        point_offset.value = point_offset.value + 100
        print(f"Point distance: {point_offset.value}")
    elif e.keycode == 82:   # keypad -
        if point_offset.value >= 200:
            #with point_offset.get_lock():
            point_offset.value = point_offset.value - 100
        else:
            point_offset.value = 100
        print(f"Point distance: {point_offset.value}")

    elif e.keysym == 't':
        #with keep_distance.get_lock():
        keep_distance.value = 1 - keep_distance.value 
        print("Toggle distance fence to: ", keep_distance.value)
    elif e.keysym == 'p':
        #with show_pos.get_lock():
        show_pos.value = 1 - show_pos.value
        print("Toggle show position to: ", show_pos.value)

    if e.keysym == 'space':
        print(">> Stop")
        current_point.value = 0
        manual_target.value = 0
        set_speed(the_connection, 0, 0, 0, 0)

def keyboard_handler():
    root = tk.Tk()
    print("K-- Keyboard loading")

    root.bind("<KeyPress>", lambda e: keyboard_keydown(e))

    #root.bind("<KeyPress>", keyboard_keyup)
    root.mainloop()

if __name__ == '__main__':

    print("RoboCamp initializing...")
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    while (the_connection.target_system == 0):
        print("D-- Checking Heartbeat")
        the_connection.wait_heartbeat()
        print("D-- Heartbeat from system (system %u component %u)" 
              % (the_connection.target_system, the_connection.target_component))

    ### Synchronized shared objects
    current_point = Value('i', 0)   # (int)  Current target point for testing
    keep_distance = Value('i', 0)   # (bool) Keep offset from target location to not hit it
    manual_target = Value('i', 0)   # (bool) Confirmation for proceeding to current target
    show_pos = Value('i', 1)        # (bool) Toggle target lat/long print in console

    point_offset = Value('i', 500)  # (int) lat/log offset from points [E-7]

    bt_lat = Value('f', 0.0)        # (float)latest bracelet gps latitude   
    bt_lon = Value('f', 0.0)        # (float)latest bracelet gps longitude
    bt_alt = Value('f', 10.0)       # (float)bracelet follow height (AGL)


    host = '192.168.0.145'
    port = 3333

    process_1 = Process(target=udp_server_handler, args=(host, port,))
    process_3 = Process(target=keyboard_handler, args=())
    #process_2 = Process(target=drone_main(the_connection), args=())

    process_1.start()   #udp_server_handler()
    process_3.start()   #keyboard_handler()
    #process_2.start()   #drone_main()

    print("D-- Drone enabled")

    while current_point.value != -1:
        while mode_arm_check(the_connection)[2] != 'GUIDED':
            print("Waiting for GUIDED")
            time.sleep(2.5)
        
        print("in GUIDED ")

        [lat, lon, terrH] = get_drone_position_v1(the_connection)
        # msg_temp = the_connection.recv_match(type='TERRAIN_REPORT', blocking=True)
        # lat = msg_temp.lat
        # lon = msg_temp.lon
        # terrH = msg_temp.terrain_height

        while mode_arm_check(the_connection)[2] == 'GUIDED':
            if   current_point.value == 1:
                if keep_distance.value == 1:
                    follow_target_offset(the_connection, lat, lon+point_offset.value, terrH + 10)
                else:
                    follow_target_set(the_connection, lat, lon+point_offset.value, terrH + 10)

            elif current_point.value == 2:
                if keep_distance.value == 1:
                    follow_target_offset(the_connection, lat, lon+2*point_offset.value, terrH + 10)
                else:
                    follow_target_set(the_connection, lat, lon+2*point_offset.value, terrH + 10)

            elif current_point.value == 3:
                if keep_distance.value == 1:
                    follow_target_offset(the_connection, lat+point_offset.value, lon+2*point_offset.value, terrH + 10)
                else:
                    follow_target_set(the_connection, lat+point_offset.value, lon+2*point_offset.value, terrH + 10)

            elif current_point.value == 4:
                if keep_distance.value == 1:
                    follow_target_offset(the_connection, lat+point_offset.value, lon+point_offset.value, terrH + 10)
                else:
                    follow_target_set(the_connection, lat+point_offset.value, lon+point_offset.value, terrH + 10)

            elif manual_target.value == 2:
                follow_target_set(the_connection, int(bt_lat.value*10**7), int(bt_lon.value*10**7), terrH + bt_alt.value)
                #manual_target.value = 0
            #time.sleep(0.5)

        #process_1.join()
        #process_2.join()
        #process_3.join()

    print("D-- RoboCamp terminated")


