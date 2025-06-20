#!/usr/bin/env python3

import math, time
from pymavlink import mavutil

import sys, argparse, getopt

set_yaw = [0,0,0]     # Target yaw
set_alt = 1     # Target altitude

cmd_parser = argparse.ArgumentParser(description='Input parameters.')
cmd_parser.add_argument('-a', type=int, help='set altitude')
cmd_parser.add_argument('-y', type=int, nargs='+',help='set yaw (angle)(rel-1/abs-0)(CCW--1/S-0/CW-1)')
cmd_parser.add_argument('-l', action='store_true', help='leave in loiter mode')

cmd_args = cmd_parser.parse_args()

if (cmd_args.a is not None):
    set_alt = cmd_args.a
    print (("Set altitude: %s") % (set_alt))

if (cmd_args.y is not None):
    set_yaw[0] = cmd_args.y[0]
    print (("Set yaw: %s") % (set_yaw[0]))
    if len(cmd_args.y) >= 2:
        if (cmd_args.y[1] == 0):
            print("Reference: Absolute")
        elif (cmd_args.y[1] == 1):
            print("Reference: Relative")
        else:
            print("Wrong reference, changing to Relative")
            cmd_args.y[1]=0
        set_yaw[1] = cmd_args.y[1]

    if len(cmd_args.y) == 3:
        if (cmd_args.y[2] == 1):
            print("Direction: Clockwise")
        elif (cmd_args.y[2] == 0):
            print("Direction: Shortest")
        elif (cmd_args.y[2] == -1):
            print("Direction: Counter Clockwise")
        else:
            print("Wrong direction, changing to Shortest")
            cmd_args.y[2]=0
        set_yaw[2] = cmd_args.y[2]   
    print(cmd_args.y)

if cmd_args.l:
    print("Will leave in LOITER mode!")
    SET_LOITER = True
else:
    SET_LOITER = False

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
    ack(the_connection, "COMMAND_ACK")

def change_yaw(the_connection):
    print("-- Changing Yaw to ", set_yaw[0])
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        115, #mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        #213, #mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
        0,
        set_yaw[0],  # Angle [deg]
        30,         # Angular speed [deg/s]
        set_yaw[2],  # Direction (-1>CCW, 0>shortest, 1>CW)
        set_yaw[1],  # Absolute/Relative (0/1)
        0,0,0    
    )
    #ack(the_connection, "COMMAND_ACK")

# Takeoff the drone
def takeoff(the_connection):
    print("-- Takeoff initialized")

    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        22, #mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,1,
        set_yaw[0],
        0,0,
        set_alt
    )
    ack(the_connection, "COMMAND_ACK")

# Acknoledgement from the Drone 
def ack(the_connection, keyword):
    print("-- Message Read " + str(the_connection.recv_match(type=keyword, blocking=True)))

def mode_arm_check(the_connection):
    '''returns states: armed, armable, mode'''
    correct_type = 0
    while (correct_type == 0):
        msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
        if not msg:
            print("No heartbeat!")
            return None
        elif msg.type == 2:
            correct_type = 1

    custom_mode = msg.custom_mode
    base_mode = msg.base_mode
    mode = mavutil.mode_mapping_acm.get(custom_mode)
    armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    armable = ((base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0) or ((base_mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED) != 0)
    
    print(custom_mode)
    print(base_mode)
    print(mode)
    print(armed)
    print(armable)
    return [armed, armable, mode]    

# Main function
if __name__ == "__main__":
    print("-- Program Started")
    
    #the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14551')
    the_connection = mavutil.mavlink_connection('/dev/ttyACM0,921600')
    #the_connection = mavutil.mavlink_connection('/dev/ttyACM0')
    
    print("-- Checking Heartbeat")
    while (the_connection.target_system == 0):
        the_connection.wait_heartbeat()

    print("-- Heartbeat from system (system %u component %u)" 
              % (the_connection.target_system, the_connection.target_component))

    state = mode_arm_check(the_connection)

    if state[2] == None:
        print("Mode error")
    elif state[2] == 'GUIDED':
        print("Continuing in: ", state[2])
    else:
        print("Abort, exit!")
        sys.exit()

    if state[0]:
        print("Already armed")

        print("Already airborne")
        sys.exit()

    elif state[1]:
        print("Armable")
        arm(the_connection)

        while (mode_arm_check(the_connection)[0] != True):
            print("Arming...")
            time.sleep(1)
    
        takeoff(the_connection) # proceed to takeoff, wait for altitude
        print("Takeoff!")
    else:
        print("Not Armable!")

    time.sleep(5)

    change_yaw(the_connection)

    print("--Done")
    