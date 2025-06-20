#!/usr/bin/env python3

from pymavlink import mavutil
import tkinter as tk
import sys
import socket
from multiprocessing import Process, Value
import time
import math

#from robocamp_drone_functions import *
from NMEA_parser import *

#def udp_server_handler(host='192.168.0.229', port=3333): # Jetson
#def udp_server_handler(host='192.168.0.145', port=3333):# sim/Putanec
def udp_server_handler():

    while True:
        data, client_adress = server_socket.recvfrom(1024)

        #print(data)
        do_send = 0

        if server_enable.value == -1:
            print("U-- server ended")
            sys.exit()

        print(f"{data} from {client_adress}")

# Wait for ping1 response from drone
def wait_for_drone(drone_adress, pings=1):
    n_pings = 0

    print(f"U-- Waiting for drone on: {drone_adress}")
    while (n_pings < pings):

        server_socket.sendto(b'ping', drone_adress)

        data, client_adress = server_socket.recvfrom(1024)

        if data == b'ping1':
            n_pings = n_pings + 1
            print(f"Received {n_pings}. ping1!")
    return 1

# Keyboard commands to send to the drone
def keyboard_keydown(e):

    if e.keysym == 'space':
        server_socket.sendto(b'stop', jetson_adress)
        print(">> Stop")

    elif e.keysym == 'a':
        server_socket.sendto(b'arm', jetson_adress)
        print(">> Try ARMING")  #arm(the_connection)

    elif e.keysym == 'g':
        server_socket.sendto(b'guided', jetson_adress)
        print(">> Set GUIDED")  #mode_send(the_connection, 4)

    elif e.keysym == 'l':
        server_socket.sendto(b'land', jetson_adress)
        print(">> Set LAND")    #mode_send(the_connection, 9)

    elif e.keysym == 'r':
        server_socket.sendto(b'rtl', jetson_adress)
        print(">> Set RTL")     #set_return(the_connection)

    elif e.keysym == 'c':
        print("K-- Keyboard closed")
        server_enable.value = -1
        sys.exit()

    elif e.keysym == 't':  
        server_socket.sendto(b'takeoff', jetson_adress)
        print("Try to takeoff...")

    elif e.keysym == 's':
        #with forward_nmea.get_lock():
        forward_nmea.value = 1 - forward_nmea.value
        print(f"Toggle NMEA forwarding to {forward_nmea.value}")

    elif e.keysym == 'p':
        server_socket.sendto(b'ping', jetson_adress)
        print(">> Sent PING")



def keyboard_handler():
    root = tk.Tk()
    print("K-- Keyboard loading")

    root.bind("<KeyPress>", lambda e: keyboard_keydown(e))

    root.mainloop()


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

    ### Synchronized shared objects
    server_enable = Value('i', 0)   # (int)  server enable
    forward_nmea = Value('i', 0)    # (bool) Toggle nmea forwarding to drone

    #host='127.0.0.1', port=4444 # Jetson
    host = '192.168.0.145'
    port = 3333 # sim/Putanec
    
    jetson_adress = '127.0.0.1', 4444

    saved_adress = ''

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((host, port))
    print(f'U-- UDP server listening on {host}:{port}')

    process_1 = Process(target=keyboard_handler, args=())
    process_2 = Process(target=udp_server_handler, args=())#(host, port,))

    process_1.start()   #keyboard_handler()

    # if wait_for_drone(jetson_adress, 3):
    #     print(f"Successfully connected with {jetson_adress}")

    process_2.start()   #udp_server_handler()

    process_2.join()

    server_socket.close()

    process_1.join()

    print("D-- RoboCamp terminated")


