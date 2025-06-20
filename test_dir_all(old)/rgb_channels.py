#!/usr/bin/env python3

import math, time
from pymavlink import mavutil
import tkinter as tk
import sys

history = []    # keypress history

def keydown(e):

    if e.keysym == 'f': #r
        print(">> Free drive")
        rc_override(the_connection, 1600)

    elif e.keysym == 'g': #p
        print(">> Search - no data")
        rc_override(the_connection, 1700)

    elif e.keysym == 'h':
        print(">> Face found")
        rc_override(the_connection, 1750)

    elif e.keysym == 'j':
        print(">> Tracking - no data")
        rc_override(the_connection, 1800)

    elif e.keysym == 'k':
        print(">> Tracking Face")
        rc_override(the_connection, 1850)


    elif e.keysym == 'space':
        print(">> RC reset")
        rc_override(the_connection, 1495)

    elif e.keysym == 'c':
        print(">> Closing")
        sys.exit()

# Set RC channel override
def rc_override(the_connection, RC12):#, ch_int, ch_val):
    #print("Overriding channel %s to %s", ch_int, ch_val)
    the_connection.mav.rc_channels_override_send(
        the_connection.target_system,
        the_connection.target_component,
        0, #65535,  # UINT16_MAX -> IGNORE
        0, #65535,  # 2
        0, #65535,  # 3
        0, #65535,  # 4
        0, #65535,  # 5
        0, #65535,  # 6
        0, #65535,  # 7
        0, #65535,  # 8
        65534,  # UINT16_MAX - 1 
        65534,
        65534,
        RC12
    )

# Acknoledgement from the Drone 
def ack(the_connection, keyword):
    print("-- Message Read " + str(the_connection.recv_match(type=keyword, blocking=True)))


if __name__ == "__main__":
    print("Mavlink Keyboard started")

    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    while (the_connection.target_system == 0):
        print("-- Checking Heartbeat")
        the_connection.wait_heartbeat()
        print("-- Heartbeat from system (system %u component %u)" 
              % (the_connection.target_system, the_connection.target_component))


    # read the keyboard with tkinter
    root = tk.Tk()
    print(">> Keyboard control")
    #root.bind_all('<Key>',key)
    root.bind("<KeyPress>", keydown)
    #root.bind("<KeyRelease>", keyup)

    root.mainloop()