#!/usr/bin/env python3

import socket
import re

import math, time
from pymavlink import mavutil
import tkinter as tk
import sys

#UDP_IP = "192.168.0.102"
UDP_PORT = 3333



# Read the content of the socat output log file 
with open('/tmp/socat_output.log', 'r') as file:
    log_content = file.read()

# Find all matches of the pattern 'PTY is /dev/pts/number'
matches = re.findall(r'PTY is (/dev/pts/\d+)', log_content)

# Get the first match
if matches:
    VIRTUAL_SERIAL_PORT = matches[0]
    print(f"Found open pts for writing: {matches[0]}")
else:
    print("No open socat pts found, aborting...")
    sys.exit()

#VIRTUAL_SERIAL_PORT = "/dev/pts/2"     # not reliable, pts changes per process

def start_udp_server(host='192.168.0.103', port=3333):
#def start_udp_server(host='192.168.43.221', port=3333): # esp32:98

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    server_socket.bind((host, port))
    print(f'UDP server listening on {host}:{port}')

    while True:

        data, client_adress = server_socket.recvfrom(1024)

        #print(data.decode('utf-8'))

        senddata(data)

        #with open(VIRTUAL_SERIAL_PORT, 'wb') as f:
            #f.write(data)
            #f.write(data.decode('utf-8'))
            #f.write(bytes(data))
            #f.write(b'\n')

def key(event):
    global whichNum
    if event.char == event.keysym:  # standard keys
        if event.keysym == 'r':
            print(">> Set RTL")
            #set_return(the_connection)

        elif event.keysym == 'l':
            print(">> Set LAND")
            #mode_land(the_connection)

        elif event.keysym == 'g':
            print(">> Set GUIDED")
            #mode_guided(the_connection)

        elif event.keysym == 'c':
            print(">> Closing")
            sys.exit()

    else:                           # non standard keys
        if event.keysym == 'Up':
            print("Up ")   # actually forward
            whichNum += 1
            print(whichNum.to_bytes(1))
            #set_speed(the_connection, 1, 0, 0, 0)
        elif event.keysym == 'Down':
            print("Down ")   # actually backward
            whichNum -= 1
            print(whichNum.to_bytes(1))
            #set_speed(the_connection, -1, 0, 0, 0)
        elif event.keysym == 'Left':
            print("Left ")
            print(whichNum.to_bytes(1))
            #set_speed(the_connection, 0, -1, 0, 0)
        elif event.keysym == 'Right':
            print("Right ")
            print(whichNum)
            senddata(whichNum.to_bytes(1))
            #set_speed(the_connection, 0, 1, 0, 0)

        elif event.keysym == 'KP_1' or event.keysym == '1':
            whichNum = 1
            print("num 1")
            senddata(b'1')
            #set_speed(the_connection, 0, 0, 0, -1.6)
        elif event.keysym == 'KP_2' or event.keysym == '2':
            whichNum = 2
            print("num 2")
            senddata(b'2')
            #set_speed(the_connection, 0, 0, 0, 1.6)
        elif event.keysym == 'KP_3' or event.keysym == '3':
            whichNum = 3
            print("num 3")
            senddata(b'3')
            #set_speed(the_connection, 0, 0, -0.5, 0)
        elif event.keysym == 'KP_4' or event.keysym == '4':
            whichNum = 4
            print("num 4")
            #senddata(b'4')
            senddata(whichNum)
            #set_speed(the_connection, 0, 0, 0.5, 0)
        elif event.keysym == 'space':
            print("Sent ")
            #print("Sent ", whichNum)

            senddata(whichNum.to_bytes(1))
            #with open(VIRTUAL_SERIAL_PORT, 'wb') as f:
            #    f.write(whichNum)

def senddata(dataToSend):

    with open(VIRTUAL_SERIAL_PORT, 'wb') as f:

        f.write(dataToSend)
        

if __name__ == "__main__":
    print("Mavlink Keyboard started")

    #the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    #while (the_connection.target_system == 0):
    #    print("-- Checking Heartbeat")
    #    the_connection.wait_heartbeat()
    #    print("-- Heartbeat from system (system %u component %u)" 
    #          % (the_connection.target_system, the_connection.target_component))
    whichNum = 0

    # read the keyboard with tkinter
    root = tk.Tk()
    print(">> Keyboard control")
    root.bind_all('<Key>',key)
    root.mainloop()

    start_udp_server()
