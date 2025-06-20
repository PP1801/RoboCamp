#!/usr/bin/env python3

import math, time
from pymavlink import mavutil
import tkinter as tk
import sys

set_gnd_speed = 2 # ground speed [m/s]
history = []    # keypress history

tmp_v = [0, 0, 0, 0]

def keyup(e):

    if e.char == e.keysym:  # standard keys
        print("?")
    else:                   # non standard keys
        if e.keysym == 'Up':
            tmp_v[0] = 0

        elif e.keysym == 'Down':
            tmp_v[0] = 0

        if e.keysym == 'Left':
            tmp_v[1] = 0

        elif e.keysym == 'Right':
            tmp_v[1] = 0 
    
        if e.keysym == 'KP_1':
            tmp_v[3] = 0

        elif e.keysym == 'KP_3':
            tmp_v[3] = 0

        if e.keysym == 'KP_5':
            tmp_v[2] = 0

        elif e.keysym == 'KP_2':
            tmp_v[2] = 0

        if e.keysym == 'KP_0' or e.keysym == 'space':
            print("stop")
            tmp_v[0] = tmp_v[1] = tmp_v[2] = tmp_v[3] = 0

        set_speed(the_connection, tmp_v[0], tmp_v[1], tmp_v[2], tmp_v[3])


def keydown(e):
    if e.keysym == 'r':
        print(">> Set RTL")
        set_return(the_connection)

    elif e.keysym == 'l':
        print(">> Set LAND")
        mode_land(the_connection)

    elif e.keysym == 'g':
        print(">> Set GUIDED")
        mode_guided(the_connection)

    elif e.keysym == 'c':
        print(">> Closing")
        sys.exit()

    else:      # non standard keys
        if e.keysym == 'Up':
            ##print("Up")   # actually forward
            #set_speed(the_connection, 1, 0, 0, 0)
            tmp_v[0] = 1
        elif e.keysym == 'Down':
            #print("Down")   # actually backward
            #set_speed(the_connection, -1, 0, 0, 0)
            tmp_v[0] = -1
        if e.keysym == 'Left':
            #print("Left")
            #set_speed(the_connection, 0, -1, 0, 0)
            tmp_v[1] = -1
        elif e.keysym == 'Right':
            #print("Right")
            #set_speed(the_connection, 0, 1, 0, 0)
            tmp_v[1] = 1 
        
        if e.keysym == 'KP_1':
            #print("")
            #set_speed(the_connection, 0, 0, 0, -1.6)
            tmp_v[3] = -0.1
        elif e.keysym == 'KP_3':
            #print("")
            #set_speed(the_connection, 0, 0, 0, 1.6)
            tmp_v[3] = 0.1
        if e.keysym == 'KP_5':
            #print("")
            #set_speed(the_connection, 0, 0, -0.5, 0)
            tmp_v[2] = -0.2
        elif e.keysym == 'KP_2':
            #print("")
            #set_speed(the_connection, 0, 0, 0.5, 0)
            tmp_v[2] = 0.2

        if e.keysym == 'KP_0' or e.keysym == 'space':
            print("stop")
            #set_speed(the_connection, 0, 0, 0, 0)
            tmp_v[0] = tmp_v[1] = tmp_v[2] = tmp_v[3] = 0

    set_speed(the_connection, tmp_v[0], tmp_v[1], tmp_v[2], tmp_v[3])
(
# def key(event):
#     tmp_v = [0, 0, 0, 0]    # temporary velocity vector
#                             # vx vy vz ywr

#    if event.char == event.keysym:  # standard keys
#         if event.keysym == 'r':
#             print(">> Set RTL")
#             set_return(the_connection)

#         elif event.keysym == 'l':
#             print(">> Set LAND")
#             mode_land(the_connection)

#         elif event.keysym == 'g':
#             print(">> Set GUIDED")
#             mode_guided(the_connection)

#         elif event.keysym == 'c':
#             print(">> Closing")
#             sys.exit()

#     else:                           # non standard keys
#         if event.keysym == 'Up':
#             ##print("Up")   # actually forward
#             #set_speed(the_connection, 1, 0, 0, 0)
#             tmp_v[0] += 1
#         elif event.keysym == 'Down':
#             #print("Down")   # actually backward
#             #set_speed(the_connection, -1, 0, 0, 0)
#             tmp_v[0] -= 1
#         if event.keysym == 'Left':
#             #print("Left")
#             #set_speed(the_connection, 0, -1, 0, 0)
#             tmp_v[1] -= 1
#         elif event.keysym == 'Right':
#             #print("Right")
#             #set_speed(the_connection, 0, 1, 0, 0)
#             tmp_v[1] += 1 
    
#         if event.keysym == 'KP_1':
#             #print("")
#             #set_speed(the_connection, 0, 0, 0, -1.6)
#             tmp_v[3] -= 1.6
#         elif event.keysym == 'KP_3':
#             #print("")
#             #set_speed(the_connection, 0, 0, 0, 1.6)
#             tmp_v[3] += 1.6
#         if event.keysym == 'KP_5':
#             #print("")
#             #set_speed(the_connection, 0, 0, -0.5, 0)
#             tmp_v[2] -= 0.5
#         elif event.keysym == 'KP_2':
#             #print("")
#             #set_speed(the_connection, 0, 0, 0.5, 0)
#             tmp_v[2] += 0.5

#         if event.keysym == 'KP_0' or event.keysym == 'space':
#             print("stop")
#             #set_speed(the_connection, 0, 0, 0, 0)
#             tmp_v[0] = tmp_v[1] = tmp_v[2] = tmp_v[3] = 0

#         set_speed(the_connection, tmp_v[0], tmp_v[1], tmp_v[2], tmp_v[3])
)
def set_speed(the_connection, vx, vy, vz, ywr):
    #print("-- Takeoff initialized")

    #the_connection.mav.set_position_target_local_ned
    #the_connection.mav.set_position_target_global_int_send(
    the_connection.mav.set_position_target_local_ned_send(
        #mavutil.mavlink.SET_POSITION_TARGET_LOCAL_NED, #84,
        0, # time_boot_ms
        the_connection.target_system,
        the_connection.target_component,
        #20, #mavutil.mavlink.MAV_FRAME_LOCAL_FRD, #20,
        #mavutil.mavlink.MAV_FRAME_LOCAL_NED, #1,
        #mavutil.mavlink.MAV_FRAME_BODY_FRD, #
        12,
        0b0000101111000111,#bitmask -> just use velocities 
        0,0,0,      # x, y, z       -> position
        vx,vy,vz,   # vx, vy, vz    -> velocities
        0,0,0,      # afx, afy, afz -> accelerations
        0, ywr     # yaw, yaw_rate
    )

# Arm the drone
(#def arm(the_connection):
#    print("-- Arming")
#
#    the_connection.mav.command_long_send(
#        the_connection.target_system,
#        the_connection.target_component,
#        400, #mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#        0,
#        1,
#        0,0,0,0,0,0
#    )
#    ack(the_connection, "COMMAND_ACK")
)

# Set mode to GUIDED
def mode_guided(the_connection):
    print("-- Changing Mode to GUIDED")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        176, #mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,
        4, #MODE_GUIDED
        0,0,0,0,0
    )
    #ack(the_connection, "COMMAND_ACK")

def mode_land(the_connection):
    print("-- Changing Mode to LAND")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        176, #mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,
        9, #MODE_LAND ?
        0,0,0,0,0
    )
    #ack(the_connection, "COMMAND_ACK")

def set_return(the_connection):
    print("-- Set RTL")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        20, #mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,0,0,0,0,0,0,0
    )
    #ack(the_connection, "COMMAND_ACK")

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
    root.bind("<KeyRelease>", keyup)

    root.mainloop()