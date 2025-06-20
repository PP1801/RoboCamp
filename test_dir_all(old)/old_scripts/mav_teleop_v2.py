#!/usr/bin/env python3

import math, time
from pymavlink import mavutil
import tkinter as tk
import sys

set_gnd_speed = 2 # ground speed [m/s]
history = []    # keypress history
send = False

tmp_v = [0, 0, 510, 0]

(
# def keyup(e):

#     if e.char == e.keysym:  # standard keys
#         print("?")
#     else:                   # non standard keys
#         if e.keysym == 'Up':
#             tmp_v[0] = 0
#             send = True

#         if e.keysym == 'Down':
#             tmp_v[0] = 0
#             send = True

#         if e.keysym == 'Left':
#             tmp_v[1] = 0
#             send = True

#         if e.keysym == 'Right':
#             tmp_v[1] = 0 
#             send = True
            
#         if e.keysym == 'KP_1':
#             tmp_v[3] = 0
#             send = True

#         if e.keysym == 'KP_3':
#             tmp_v[3] = 0
#             send = True

#         if e.keysym == 'KP_5':
#             tmp_v[2] = 510
#             send = True

#         if e.keysym == 'KP_2':
#             tmp_v[2] = 510
#             send = True

#         #if e.keysym == 'KP_0' or e.keysym == 'space':
#         #    print("stop")
#         #   tmp_v[0] = tmp_v[1] = tmp_v[2] = tmp_v[3] = 0

#         if (send): 
#             manual_mav(the_connection, tmp_v[0], tmp_v[1], tmp_v[2], tmp_v[3], 0)
#             print(tmp_v)
#             send = False
#         #set_speed(the_connection, tmp_v[0], tmp_v[1], tmp_v[2], tmp_v[3])
)

def keydown(e):
    send = False
    if e.keysym == 'r':
        print(">> Set RTL")
        set_return(the_connection)

    elif e.keysym == 'l':
        print(">> Set LAND")
        mode_send(the_connection, 9)
        #mode_land(the_connection)

    elif e.keysym == 'g':
        print(">> Set GUIDED")
        mode_send(the_connection, 4)
        #mode_guided(the_connection)

    elif e.keysym == 'a':
        print(">> Try ARMING")
        arm(the_connection)

    elif e.keysym == 'p':
        print(">> Set POSHOLD")
        mode_send(the_connection, 16)
        #mode_poshold(the_connection)
    
    elif e.keysym == 't':
        print(">> Set LOITER")
        mode_send(the_connection, 5)
        #mode_loiter(the_connection)

    elif e.keysym == 's':
        print(">> Set STABILIZE")
        mode_send(the_connection, 0)
        #mode_stabilize(the_connection)

    elif e.keysym == 'm':
        print("named_value_int_send")
        #timestamp = int(time.time() * 1000)
        the_connection.mav.named_value_int_send(
            #the_connection.target_system,
            #the_connection.target_component,
            #252, #mavutil.mavlink.NAMED_VALUE_FLOAT
            #0, # confirmation
            0, #timestamp, # time_boot_ms
            b'p', # name char[10]
            27 # value int32_t
        )
    
    elif e.keysym == 'd':
        print("debug_send")
        the_connection.mav.debug_send(
            0, #timestamp in seconds (float) !
            5, # The debug value (float)
            2     # Index (uint8) message identifier
        )

    elif e.keysym == 'c':
        print(">> Closing")
        sys.exit()

    else:      # non standard keys
        if e.keysym == 'Up':
            tmp_v[0] = 250
            send = True

        elif e.keysym == 'Down':
            tmp_v[0] = -250
            send = True

        if e.keysym == 'Left':
            tmp_v[1] = -250
            send = True

        elif e.keysym == 'Right':
            tmp_v[1] = 250
            send = True

        if e.keysym == 'KP_1':
            tmp_v[3] = -150
            send = True

        elif e.keysym == 'KP_3':
            tmp_v[3] = 150
            send = True

        if e.keysym == 'KP_5':
            tmp_v[2] = 700  # 500 + 100
            send = True

        elif e.keysym == 'KP_2':
            tmp_v[2] = 350  # 500 - 100
            send = True            

        if e.keysym == 'KP_0' or e.keysym == 'space':
            print("stop")
            #set_speed(the_connection, 0, 0, 0, 0)
            tmp_v[0] = tmp_v[1] = tmp_v[3] = 0
            tmp_v[2] = 510
            send = True

    if (send):
        #manual_mav(the_connection, tmp_v[0], tmp_v[1], tmp_v[2], tmp_v[3], 0)
        set_speed(the_connection, tmp_v[0]/1000, tmp_v[1]/1000, (tmp_v[2]-500)/500, tmp_v[3]/250)
        
        print(tmp_v)
        send = False

def manual_mav(the_connection, x, y, z, r, butt):
    the_connection.mav.manual_control_send(
        the_connection.target_system,
        x,  # x € [-1000, 1000] -> 0
        y,  # y € [-1000, 1000] -> 0
        z,  # z € [    0, 1000] -> 500
        r,  # r € [-1000, 1000] -> 0
        butt   # button
    )

def set_speed(the_connection, vx, vy, vz, ywr):
    #print("-- Takeoff initialized")

    #the_connection.mav.set_position_target_local_ned
    #the_connection.mav.set_position_target_global_int_send(
    the_connection.mav.set_position_target_local_ned_send(
        #mavutil.mavlink.SET_POSITION_TARGET_LOCAL_NED, #84,
        0, # time_boot_ms
        0, #the_connection.target_system, #0,
        0, #the_connection.target_component, #0,
        #20, #mavutil.mavlink.MAV_FRAME_LOCAL_FRD, #20,
        #mavutil.mavlink.MAV_FRAME_LOCAL_NED, #1,
        #mavutil.mavlink.MAV_FRAME_BODY_FRD, #
        #12,
        8, #mavutil.mavlink.MAV_FRAME_BODY_NED
        0b0000011111000111,#bitmask -> just use velocities 
        0,0,0,      # x, y, z       -> position
        vx,vy,vz,   # vx, vy, vz    -> velocities   
        0,0,0,      # afx, afy, afz -> accelerations
        0, ywr     # yaw, yaw_rate
    )

def set_return(the_connection):
    #print("-- Set RTL")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        20, #mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,0,0,0,0,0,0,0
    )
    #ack(the_connection, "COMMAND_ACK")

# Arm the drone
def arm(the_connection):
   print("-- Arming")

   the_connection.mav.command_long_send(
       the_connection.target_system,
       the_connection.target_component,
       400, #mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
       0,
       1,
       0,0,0,0,0,0
   )
   #ack(the_connection, "COMMAND_ACK")

def mode_send(the_connection, this_mode):
    """
    0 -> STABILIZE
    4 -> GUIDED
    5 -> LOITER
    9 -> LAND
    16 -> POSHOLD
    """
    #print("-- Changing Mode to GUIDED")
    the_connection.mav.command_long_send(
        1, #the_connection.target_system,
        1, #the_connection.target_component,
        176, #mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,
        this_mode,
        0,0,0,0,0
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
    #root.bind("<KeyRelease>", keyup)

    root.mainloop()