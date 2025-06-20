#!/usr/bin/env python3

from pymavlink import mavutil
import sys
import socket
from multiprocessing import Process, Value
import time
import math

from robocamp_drone_functions import *
from NMEA_parser import *

#def udp_server_handler(host='192.168.0.229', port=3333): # Jetson
#def udp_server_handler(host='192.1d68.0.145', port=3333):# sim/Putanec
def udp_server_handler():
    # server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # server_socket.bind((host, port))
    # print(f'U-- UDP server listening on {host}:{port}')

    while True:
        data, client_adress = server_socket.recvfrom(1024)

        print(data)

        if current_point.value == -1:
            print("U-- server ended")
            sys.exit()

        if (data == b'stop'):
            print(">> Stop")
            with current_point.get_lock():
                current_point.value = 0
            with manual_target.get_lock():
                manual_target.value = 0
            set_speed(the_connection, 0, 0, 0, 0)

        elif (data == b'ping'):
            print(f"Echo ping1 back to {client_adress}")
            server_socket.sendto(b'ping1', client_adress)

        # elif check_is_nmea(data):
        #     print(f"nmea:{data}")
        #     try:
        #         parsed_data = parse_gngga(data, 0)
        #         #print(f"parsed:{parsed_data}")

        #         tmp_lat, tmp_lon = parsed_data['latitude'], parsed_data['longitude']
                
        #         if (tmp_lat is not None) and (tmp_lon is not None):
        #             if manual_target.value == 1:
        #                 bt_lat.value, bt_lon.value = tmp_lat, tmp_lon
        #                 manual_target.value = 2
        #                 print(f"Try following-> lat: {bt_lat.value:.7f}\tlon: {bt_lon.value:.7f}")
        #             if show_pos.value == 1:
        #                 print(f"lat: {tmp_lat:.7f}\tlon: {tmp_lon:.7f}")

        #     except:
        #         print("bad NMEA - skipped")

        elif check_is_follow(data):          
            parsed_data = parse_follow(data)
            #print(f"parsed:{parsed_data}")

            tmp_lat, tmp_lon = float(parsed_data['latitude']), float(parsed_data['longitude'])
            bracelet_id = parsed_data['bracelet']
            tmp_alt, tmp_off = float(parsed_data['altitude']), float(parsed_data['offset'])
            #print(f"{bracelet_id}, {tmp_lat}, {tmp_lon}")
            
            if (tmp_lat is not None) and (tmp_lon is not None):
                with bt_lat.get_lock():
                    bt_lat.value = tmp_lat
                with bt_lon.get_lock():
                    bt_lon.value = tmp_lon
                with bt_alt.get_lock():
                    bt_alt.value = tmp_alt
                with bt_off.get_lock():
                    bt_off.value = tmp_off

                print(f"Saving {bracelet_id} at: {bt_lat.value:.7f}, {bt_lon.value:.7f}")

                if manual_target.value == 1:
                    with manual_target.get_lock():
                        manual_target.value = 2
                # if show_pos.value == 1:
                #     print(f"lat: {tmp_lat:.7f}\tlon: {tmp_lon:.7f}")
            #except:
            #    print("bad NMEA - skipped")

        elif check_is_cmd(data):
            #print(data)
            parsed_data = parse_cmd(data)
            command, bracelet_id = parsed_data['cmd'], parsed_data['bip']
    
            #print(f"got {command} from {bracelet_id}")

            do_send = 0

            ### Bracelet commands from robocamp_main_server.py
            if  (command == 'search'):
                print("Face search request:", bracelet_id)
                #server_socket.sendto(b'search1', client_adress)
                #current_point.value = 1
                do_send = 1

            elif (command == 'track'):
                print("Face tracking request:", bracelet_id)
                #server_socket.sendto(b'track1', client_adress)
                #current_point.value = 2
                do_send = 1

            elif (command == 'gps'):
                print("GPS location request:", bracelet_id)
                #server_socket.sendto(b'gps1', client_adress)
                #current_point.value = 3

                return_data = f"#{command}1,{bracelet_id}".encode('utf-8')
                server_socket.sendto(return_data, client_adress)

                if manual_target.value == 0:
        
                    print("Auto takeoff...")

                    alpha = 0
                    #print(f"bt_lat:{bt_lat.value:.7f} bt_lon:{bt_lon.value:.7f}")
                    if bt_lat.value > 0.0 and bt_lon.value > 0.0:
                        [tmp_lat, tmp_lon, heading] = get_drone_position(the_connection)
                        [target_lat, target_lon, dist, alpha] = haversine_calculator(tmp_lat, tmp_lon, bt_lat.value, bt_lon.value)
                        [target_lat, target_lon, dist, alpha] = haversine_calculator(tmp_lat, tmp_lon, int(bt_lat.value*10**7), int(bt_lon.value*10**7))

                        #print("tgt_lat: ", target_lat), print("tgt_lon: ", target_lon), 
                        print("fdist_m: ", dist), print("alpha: ", alpha)

                        alpha = 90 - int(alpha*180.0/math.pi)   # convert from rad to deg
                        if alpha < 0:                           #  y  rad deg   N
                            alpha = 360 + alpha                 #  o-x        W-o

                    with block_main.get_lock():
                        block_main.value = 1

                    _takeoff_ack = simple_takeoff(the_connection, set_yaw=alpha)
                    if _takeoff_ack == 1:   # successful takeoff
                        with block_main.get_lock():
                            block_main.value = 0

                        with manual_target.get_lock():
                            manual_target.value = 1
                        return_data = f"s{_takeoff_ack}".encode('utf-8')    # success
                    else:
                        return_data = f"f{_takeoff_ack}".encode('utf-8')    # fail
                    #elif _takeoff_ack == 0: # not in guided
                    #elif _takeoff_ack == -1: # not armable
                    #elif _takeoff_ack == -2: # mode error
                    
                    server_socket.sendto(return_data, client_adress)
                    
                elif manual_target.value == 1:
                    print(f"Continue following")

            elif (command == 'off'):
                print("OFF request:",bracelet_id)
                #current_point.value = 0
                do_send = 1
                with manual_target.get_lock():
                    manual_target.value = 0

            else:
                print(command)

            if do_send == 1:
                return_data = f"#{command}1,{bracelet_id}".encode('utf-8')
                server_socket.sendto(return_data, client_adress)

        ### Keyboard commands from robocamp_main_server.py
        else:
            if (data == b'arm'):
                print(">> Try ARMING")
                arm(the_connection)

            elif (data == b'takeoff'):
                print("Try to takeoff...")

                alpha = 0
                print(f"bt_lat:{bt_lat.value:.7f} bt_lon:{bt_lon.value:.7f}")
                if bt_lat.value > 0.0 and bt_lon.value > 0.0:
                    [tmp_lat, tmp_lon, heading] = get_drone_position(the_connection)
                    [target_lat, target_lon, dist, alpha] = haversine_calculator(tmp_lat, tmp_lon, bt_lat.value, bt_lon.value)
                    #print("tgt_lat: ", target_lat), print("tgt_lon: ", target_lon), 
                    print("fdist_m: ", dist), print("alpha: ", alpha)

                    alpha = 90 - int(alpha*180.0/math.pi)   # convert from rad to deg
                    if alpha < 0:                           #  y  rad deg   N
                        alpha = 360 + alpha                 #  o-x        W-o

                with block_main.get_lock():
                    block_main.value = 1
                if simple_takeoff(the_connection, set_yaw=alpha):
                    with block_main.get_lock():
                        block_main.value = 0

            elif(data == b'guided'):
                print(">> Set GUIDED")
                mode_send(the_connection, 4)

            elif(data == b'loiter'):
                print(">> Set LOITER")
                mode_send(the_connection, 5)

            elif (data == b'land'):
                print(">> Set LAND")
                mode_send(the_connection, 9)

            elif (data == b'rtl'):
                print(">> Set RTL")
                set_return(the_connection)

            elif (data == b'posr'):
                manual_target.value = 1
                current_point.value = 0

            elif (data == b'p1'):
                print("point 1")
                current_point.value = 1

            elif (data == b'p2'):
                print("point 2")
                current_point.value = 2

            elif (data == b'p3'):
                print("point 3")
                current_point.value = 3

            elif (data == b'p4'):
                print("point 4")
                current_point.value = 4

            elif (data == b'lin+'):
                with point_offset.get_lock():
                    point_offset.value = point_offset.value + 100
                print(f"Point distance: {point_offset.value}")

            elif (data == b'lin-'):
                if point_offset.value >= 200:
                    with point_offset.get_lock():
                        point_offset.value = point_offset.value - 100
                else:
                    point_offset.value = 100
                print(f"Point distance: {point_offset.value}")

            # elif (data == b'fence'):
            #     with keep_distance.get_lock():
            #         keep_distance.value = 1 - keep_distance.value 
            #     print("Toggle distance fence to: ", keep_distance.value)

            elif (data.startswith(b'fence')):
                with keep_distance.get_lock():
                    keep_distance.value = int(data[5]-48) 
                print("Toggle distance fence to: ", keep_distance.value)


            #elif (data == b'poss'):
            elif (data.startswith(b'poss')):
                with show_pos.get_lock():
                    show_pos.value = int(data[4]-48)
                #show_pos.value = 1 - show_pos.value
                print("Toggle show position to: ", show_pos.value)

            elif (data == b'home'):
                print("return home")

def check_is_cmd(message):
    decoded_message = message.decode('ascii').strip()
    return decoded_message.startswith('#')

def parse_cmd(message):
    # Remove the leading '#' and split the message by '>'
    #parts = message.decode('ascii').strip().split('>')
    parts = message[1:].decode('ascii').strip().split(',')

    # Extract fields
    command_id = parts[0]  # command string/ request   removed
    braclet_id = parts[1]  # braclet id    / address

    # Return parsed data
    return {
        "cmd": command_id,
        "bip": braclet_id,
    }


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
    show_pos = Value('i', 0)        # (bool) Toggle target lat/long print in console

    point_offset = Value('i', 500)  # (int) lat/log offset from points [E-7]

    bt_lat = Value('d', 0.0)       # (dfloat) latest bracelet gps latitude   
    bt_lon = Value('d', 0.0)       # (dfloat) latest bracelet gps longitude
    bt_alt = Value('f', 10.0)       # (float) bracelet follow height (AGL) [m]
    bt_off = Value('f', 1.5)        # (float) bracelet offset [m]

    block_main = Value('i', 0)      # (bool) main loop blocking (to catch messages more easily)

    host = '127.0.0.1'
    port = 4444 # Jetson
    #host='192.168.0.145', port=3333 # sim/Putanec
    main_server_address = '192.168.0.145', 3333 # Using dedicated robocamp1 Wi-Fi router
    #jetson_adress = 1

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((host, port))
    print(f'U-- UDP server listening on {host}:{port}')

    process_1 = Process(target=udp_server_handler, args=())#(host, port,))
    #process_3 = Process(target=keyboard_handler, args=())
    #process_2 = Process(target=drone_main(the_connection), args=())

    process_1.start()   #udp_server_handler()
    #process_3.start()   #keyboard_handler()
    #process_2.start()   #drone_main()

    print("D-- Drone enabled")
    print(f"Sending ping1 to {main_server_address}")
    try:
        server_socket.sendto(b'ping1', main_server_address)
    except:
        print("Server not online")

    while current_point.value != -1:
        [lat, lon, terrH] = get_drone_position_v1(the_connection)
        print(f"D-- got pos -> terrH: {terrH:.2f}\tbt_alt: {bt_alt.value}")
        
        print("D-- Waiting for GUIDED")
        while mode_arm_check(the_connection)[2] != 'GUIDED':
            time.sleep(2.5)
        
        print("D-- in GUIDED ")

        # msg_temp = the_connection.recv_match(type='TERRAIN_REPORT', blocking=True)
        # lat = msg_temp.lat
        # lon = msg_temp.lon
        # terrH = msg_temp.terrain_height

        #while (mode_arm_check(the_connection)[2] == 'GUIDED' and block_main.value != 1):
        while (mode_arm_check(the_connection)[2] == 'GUIDED'):
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
                if keep_distance.value == 1:
                    follow_target_offset(the_connection, int(bt_lat.value*10**7), int(bt_lon.value*10**7), terrH + bt_alt.value, radius=bt_off.value)
                else:
                    follow_target_set(the_connection, int(bt_lat.value*10**7), int(bt_lon.value*10**7), terrH + bt_alt.value)
                manual_target.value = 1

                print(f"D-- following...")

                #print(f"terrH: {terrH}\tbt_alt: {bt_alt.value}")
            #time.sleep(0.5)

            while (block_main.value == 1):
                time.sleep(0.5)

        while (block_main.value == 1):
            time.sleep(0.5)

        #process_1.join()
        #process_2.join()
        #process_3.join()

    print("D-- RoboCamp terminated")