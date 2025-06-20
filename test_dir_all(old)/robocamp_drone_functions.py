#!/usr/bin/env python3

### name:           robocamp_drone_functions.py
### description:    ROBOCAMP pymavlink functions
### 
### date/updated:   28.01.2025.
### author:         Patrik Putanec
###
### used in:    * robocamp_main_server.py
###             * robocamp_jetson_server.py
###             * robocamp_bracelet_server.py
###             * robocamp_server.py

import time
import math
from pymavlink import mavutil
from haversine_calc import haversine_calculator

### DRONE FUNCTIONS
# Arm the drone
def arm(the_connection):
   print("D-- Arming")

   the_connection.mav.command_long_send(
       the_connection.target_system,
       the_connection.target_component,
       400, #mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
       0,
       1,
       0,0,0,0,0,0
   )
   #ack(the_connection, "COMMAND_ACK")

# Acknoledgement from the Drone 
def ack(the_connection, keyword):
    print("D-- Message Read " + str(the_connection.recv_match(type=keyword, blocking=True)))

# Update follow target
def follow_target_set(the_connection, lat, lon, alt):   # follow_edit_v3
    #timestamp = int(time.time() * 1000)
    the_connection.mav.set_position_target_global_int_send(
        1, #timestamp,          # Timestamp (UNIX epoch time or system time in ms)
        the_connection.target_system,
        the_connection.target_component,
        0, #mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        ( # ignore everything except x, y, & z positions
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            #mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            #mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ),
        lat,    # Latitude (degrees * 1E7)
        lon,    # Longitude (degrees * 1E7)
        alt,    # Altitude in meters -AMSL!!!
        0, 0, 0, # vx, vy, vz
        0, 0, 0, # afx, afy, afz
        0, 0,    # yaw, yawrate
    )

# Update follow target with haversine distance offset
def follow_target_offset(the_connection, true_lat, true_lon, alt, radius=1.5):
    [tmp_lat, tmp_lon, heading] = get_drone_position(the_connection)
    #print("tmp_lat: ", tmp_lat), print("tmp_lon: ", tmp_lon), print("terrH: ", terrH)
    
    [target_lat, target_lon, dist, alpha] = haversine_calculator(tmp_lat, tmp_lon, true_lat, true_lon, radius)
    #print("tgt_lat: ", target_lat), print("tgt_lon: ", target_lon), 
    print(f"fdist_m: {dist:.2f} alpha: {alpha:.2f}")

    alpha = 90 - int(alpha*180.0/math.pi)   # convert from rad to deg
    if alpha < 0:                           #  y  rad deg   N
        alpha = 360 + alpha                 #  o-x        W-o
    
    if dist >= 1.1*radius:
        follow_target_set(the_connection, int(target_lat), int(target_lon), alt)
    elif abs(alpha - heading) > 2:
        change_yaw(the_connection, alpha)      

# Get Drone position from terrain report
def get_drone_position_v1(the_connection):
    """returns drone lat, lon and terrain height"""
    msg_temp = the_connection.recv_match(type='TERRAIN_REPORT', blocking=True)
    lat = msg_temp.lat
    lon = msg_temp.lon
    terrH = msg_temp.terrain_height
    return [lat, lon, terrH]

def get_drone_position(the_connection):
    """returns drone lat, lon and heading"""
    msg_temp = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg_temp.lat
    lon = msg_temp.lon
    alt = msg_temp.alt
    rel_alt = msg_temp.relative_alt
    hdg = int(msg_temp.hdg/100)
    #print(rel_alt, hdg)
    return [lat, lon, hdg]

# Change Drone Yaw, input angle in degrees
def change_yaw(the_connection, alpha, direction=0, frame=0):
    # input alpha is in degrees!
    print("D-- Changing Yaw to ",alpha)
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        115, #mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        #213, #mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
        0,
        #int(alpha*180.0/math.pi),  # Angle [rad]->[deg]
        alpha,      # Angle [deg]
        30,         # Angular speed [deg/s]
        direction,  # Direction (-1>CCW, 0>shortest, 1>CW)
        frame,      # Absolute/Relative (0/1)
        0,0,0    
    )
    #ack(the_connection, "COMMAND_ACK")

# Set Drone speed in local NED
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

# Set Drone Mode by number
def mode_send(the_connection, this_mode):
    """0 STABILIZE 
    \n 4 GUIDED
    \n 5 LOITER
    \n 9 LAND
    \n 16 POSHOLD"""
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

# Do RTL
def set_return(the_connection):
    #print("-- Set RTL")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        20, #mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,0,0,0,0,0,0,0
    )
    #ack(the_connection, "COMMAND_ACK")

# Check for drone states
def mode_arm_check(the_connection):
    """returns states: armed, armable, mode"""
    correct_type = 0
    clean_buffer = 0
    while (correct_type == 0):
        msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
        if not msg:
            print("No heartbeat!")
            return None
        elif msg.type == 2:
            clean_buffer = clean_buffer + 1
            if (clean_buffer > 2):
                correct_type = 1
    
    custom_mode = msg.custom_mode
    base_mode = msg.base_mode
    mode = mavutil.mode_mapping_acm.get(custom_mode)
    armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    armable = ((base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0) or ((base_mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED) != 0)

    #print(custom_mode)
    #print(base_mode)
    #print(mode)
    #print(armed)
    #print(armable)
    return [armed, armable, mode]

# Takeoff the drone
def takeoff(the_connection, set_alt=10, set_yaw=0):
    print("D-- Takeoff initialized")

    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        22, #mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,1,
        set_yaw,
        0,0,
        set_alt
    )
    #ack(the_connection, "COMMAND_ACK")
    print(str(the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)))



# # Takeoff procedure
# def takeoff_master(the_connection):
#     arm(the_connection)

#     arm_int = 0
#     while (mode_arm_check(the_connection)[0] != True):
#         print("Arming...")
#         time.sleep(0.5)
#         arm_int = arm_int + 1
#         if (arm_int > 5):
#             print("Failed to arm")
#             return 0

#     takeoff(the_connection) # proceed to takeoff, wait for altitude
#     print("Takeoff!")
#     return 1

def simple_takeoff(the_connection, set_alt=10, set_yaw=0, _self=None):

    def comment(_text):
        if _self is not None:
            _self.get_logger(_text)
        else:
            print(_text)

    state = mode_arm_check(the_connection)

    if state[2] == None:
        comment("Mode error")
        return 0    # -2 -> complete mode error
    elif state[2] == 'GUIDED':
        #comment("Continuing in: ", state[2])
        pass
    else:
        comment("Not in GUIDED")
        return 0    # 0 -> not in guided

    if state[0]:
        comment("Already armed")
        comment("Probably airborne")

        # change_yaw(the_connection, set_yaw)
        # time.sleep(1)
        return 1

    elif state[1]:
        comment("Armable")
        arm(the_connection)

        arm_int = 0
        while (mode_arm_check(the_connection)[0] != True):
            comment("Arming...")
            time.sleep(1)
            arm_int = arm_int + 1
            if (arm_int > 5):
                comment("Failed to arm")
                return 0
        takeoff(the_connection, set_alt, set_yaw) # proceed to takeoff, wait for altitude
        comment("Takeoff!")

    else:
        comment("Not Armable!")
        return 0    # -1 -> not armable
    
    altitude = 0
    while((altitude <= set_alt-0.5) or (altitude >= set_alt+0.5)):
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        altitude = msg.relative_alt / 1000.0
        text = f"altitude: {altitude}" 
        comment(text)

    change_yaw(the_connection, set_yaw)

    comment("D-- Done")

    return 1

def check_is_follow(message):
    """Follow commands start with '%'"""
    try:
        decoded_message = message.decode('ascii').strip()
        return decoded_message.startswith('%')
    except:
        return 0

def parse_follow(follow_message):
    """"""
    parts = follow_message.decode('ascii').strip('%').split(',')

    return {
        "bracelet" : parts[0],  # bracelet ID: 
        "latitude" : parts[1],
        "longitude": parts[2],
        "altitude" : parts[3],
        "offset"   : parts[4],
    }

def check_is_cmd(message):
    """Commands start with '#'"""
    try:
        decoded_message = message.decode('ascii').strip()
        return decoded_message.startswith('#')
    except:
        return 0

def parse_cmd(message):
    """Commands contain strings: command,bracelet_id, like:
    \n b'#off,192.168.0.216'"""
    # Remove the leading '#' and split the message by ','
    #parts = message.decode('ascii').strip().split(',')
    parts = message[1:].decode('ascii').strip().split(',')

    # Extract fields
    command_id = parts[0]  # command string/ request   removed
    braclet_id = parts[1]  # braclet id    / address

    # Return parsed data
    return {
        "cmd": command_id,
        "bip": braclet_id,
    }

def check_is_ina(message):
    """INA current, voltage and power measurement, like:
    \n b'ina,168.75,3891.25,650.00'"""
    try:
        decoded_message = message.decode('ascii').strip()
        return decoded_message.startswith("ina")
    except:
        return 0