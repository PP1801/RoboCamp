#!/usr/bin/env python3

import math, time
from pymavlink import mavutil

# Class for formating the Mission Item
class mission_item:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = 3 #mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT # Use Global Longitude and Latitude for position data
        self.command = 16 #mavutil.mavlink.MAV_CMD_NAV_WAYPOINT        # Move to the waypoint
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 1.00
        self.param3 = 2.00
        self.param4 = math.nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0 # MAV_MISSION_TYPE value for MAV_MISSION_TYPE_MISSON

target_altitude = 10    # altitude for takeoff

# Takeoff the drone
def takeoff(the_connection):
    print("-- Takeoff initialized")

    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        22, #mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,1,
        math.nan,
        0,0,
        target_altitude
    )
    ack(the_connection, "COMMAND_ACK")

# Upload the mission items to the drone
def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("-- Sending Message out")

    the_connection.mav.mission_count_send(
        the_connection.target_system,
        the_connection.target_component,
        n,
        0
    )

    ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:  # Mission item created based on the Mavlink Message protocol
        print("-- Creating a Waypoint")

        the_connection.mav.mission_item_send(
            the_connection.target_system,   # Target System
            the_connection.target_component,# Target Component
            waypoint.seq,                   # Sequence
            waypoint.frame,                 # Frame
            waypoint.command,               # Command
            waypoint.current,               # Current
            waypoint.auto,                  # Autocontinue
            waypoint.param1,                # Hold Time
            waypoint.param2,                # Accept Radius
            waypoint.param3,                # Pass Radius
            waypoint.param4,                # Yaw
            waypoint.param5,                # Local X
            waypoint.param6,                # Local Y
            waypoint.param7,                # Local Z
            waypoint.mission_type           # Mission Type
        )

    if waypoint != mission_items[n-1]:
        ack(the_connection, "MISSION_REQUEST")

    ack(the_connection, "MISSION_ACK")

# Check if the drone is armed or armable
def is_drone_armable_or_armed(the_connection):
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)    #todo: check timeout
    if not msg:
        print("No heartbeat received.")
        return None

    base_mode = msg.base_mode
    armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    armable = ((base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0) or ((base_mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED) != 0)
    return [armed, armable]

# Get current mode of the Drone
def get_drone_mode(the_connection):
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)    #todo: check timeout
    if not msg:
        print("No heartbeat received.")
        return None
    
    custom_mode = msg.custom_mode
    #base_mode = msg.base_mode
    mode = mavutil.mode_mapping_acm.get(custom_mode)
    return mode

# Acknoledgement from the Drone 
def ack(the_connection, keyword):
    print("-- Message Read " + str(the_connection.recv_match(type=keyword, blocking=True)))

# Set mode to CIRCLE ?
def mode_circle(the_connection):
    print("-- Changing Mode to GUIDED")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        176, #mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,
        7, #mode_mapping_acm: CIRCLE
        0,0,0,0,0
    )
    ack(the_connection, "COMMAND_ACK")

def set_circle(the_connection, lat, log):
    print("-- Setting CIRCLE")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        4001, #MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE
        0,
        1,
        2, # Radius in CIRCLE_MODE
        0,0,0,
        lat,    # target latitude of center degE7
        log     # target longitude of center degE7
    )

# Set RC channel override
def throttle_override(the_connection, RC3):#, ch_int, ch_val):
    #print("Overriding channel %s to %s", ch_int, ch_val)
    the_connection.mav.rc_channels_override_send(
        the_connection.target_system,
        the_connection.target_component,
        65535,  # UINT16_MAX -> IGNORE
        65535,
        RC3,   # RC-3 -> THROTTLE ->
        65535,
        65535,
        65535,
        65535,
        65535
    )

# Follow mode
def follow_edit(the_connection, lat, lon):
    the_connection.mav.set_position_target_global_int_send(
        0,  # timestamp
        the_connection.target_system,  # target system_id
        the_connection.target_component, # target component id
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE, # |
        # mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
        # mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        # mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
        lat, #int(targetpos.lat * 1.0e7),  # lat
        lon, #int(targetpos.lng * 1.0e7),  # lon
        15,  # alt
        1,  # vx
        1,  # vy
        0,  # vz
        0,  # afx
        0,  # afy
        0,  # afz
        0,  # yaw
        0   # yawrate
    )       
    
def follow_edit_v2(the_connection, lat, lon):
    timestamp = int(time.time() * 1000)
    the_connection.mav.follow_target_send(
        1, #timestamp,          # Timestamp (UNIX epoch time or system time in ms)
        1, #mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        #mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        #mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        #mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        #mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        #mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE,   # Motion capabilities bitmask
        lat,                # Latitude (degrees * 1E7)
        lon,                # Longitude (degrees * 1E7)
        15,                # Altitude in meters
        [0.0, 0.0, 0.0], #velocity, # Velocity in NED frame (m/s)
        [0.0, 0.0, 0.0], #acceleration,       # Acceleration in NED frame (m/s^2)
        [1.0, 0.0, 0.0, 0.0], #attitude_q,         # Attitude quaternion
        [0.0, 0.0, 0.0], #angular_rates,      # Body angular rates (rad/s)
        [0.0, 0.0, 0.0], #position_cov,       # Position covariance matrix
        0, #custom_state        # Custom state (if applicable)        
    )
    #ack(the_connection, "COMMAND_ACK")

def follow_edit_v3(the_connection, lat, lon, alt):
    timestamp = int(time.time() * 1000)
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
        0,  # vx
        0,  # vy
        0,  # vz
        0,  # afx
        0,  # afy
        0,  # afz
        0,  # yaw
        0,  # yawrate
    )


# Main function
if __name__ == "__main__":
    print("-- Program Started")
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
    # 14540 Gazebo - navodno

    while (the_connection.target_system == 0):
        print("-- Checking Heartbeat")
        the_connection.wait_heartbeat()
        print("-- Heartbeat from system (system %u component %u)" 
              % (the_connection.target_system, the_connection.target_component))
        
    mission_waypoints = []

    cor = 3
    #mission_waypoints.append(mission_item(0, 0, 45.7953420, 15.9730817, 15)) # Above takeoff point
    #mission_waypoints.append(mission_item(0, 0, 45.7952364, 15.9730643, 10)) # Above destination point
    #mission_waypoints.append(mission_item(1, 0, 45.7952364, 15.9730643, 15))  # Destination point
    mission_waypoints.append(mission_item(0, 0, 45.7953420, 15.9730817, 10)) # Above takeoff point
       
    mission_waypoints.append(mission_item(4-cor, 0, 45.795398, 15.973256, 15))  # Ograda ulaz
    mission_waypoints.append(mission_item(5-cor, 0, 45.795371, 15.973357, 15))  # Miramarska ulaz
    mission_waypoints.append(mission_item(6-cor, 0, 45.796084, 15.973836, 15))  # Miramarska parking
    #mission_waypoints.append(mission_item(7-cor, 0, 45.796185, 15.974452, 15))  # Parking prilaz
    #mission_waypoints.append(mission_item(8-cor, 0, 45.796378, 15.974511, 15))  # Parking sredina

    #mavutil.set_mode()

    # check mode, change to GUIDED, wait for GUIDED
    current_mode = get_drone_mode(the_connection)
    if current_mode == None:
        print("Mode error")
    elif current_mode == 'GUIDED':
        print("Continuing in: ",current_mode)
    else:
        print("Change from ", current_mode," to GUIDED")
        
        mavutil.mavfile.set_mode_apm(the_connection, 4)
        ack(the_connection, "COMMAND_ACK")
        #mode_guided(the_connection)


    # check if ARMED, change to ARMED, wait for ARMED
    state = is_drone_armable_or_armed(the_connection)
    #print("state: ", state)
    if state[0]:
        print("Already armed")

        the_connection.mav.mission_clear_all_send(
            the_connection.target_system,
            the_connection.target_component,
            0 
        )

        msg_temp = the_connection.recv_match(type='TERRAIN_REPORT', blocking=True)
        temp_waypoints = []
        temp_waypoints.append(mission_item(0, 0, msg_temp.lat*(10**-7), msg_temp.lon*(10**-7), 10))
        temp_waypoints.append(mission_item(1, 0, msg_temp.lat*(10**-7), msg_temp.lon*(10**-7), 10))
        
        upload_mission(the_connection, temp_waypoints)

        mavutil.mavfile.set_mode_auto(the_connection)

    elif state[1]:
        print("Armable")
        mavutil.mavfile.arducopter_arm(the_connection)
        ack(the_connection, "COMMAND_ACK")

        while (is_drone_armable_or_armed(the_connection)[0] != True):
            time.sleep(1)
    
        # proceed to takeoff
        takeoff(the_connection)

    # wait for target_altitude
    altitude = 0
    while(altitude <= target_altitude-0.5):
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        altitude = msg.relative_alt / 1000.0
        print("altitude: ", altitude)


    upload_mission(the_connection, mission_waypoints)
  
    mavutil.mavfile.set_mode_auto(the_connection) #-> start_mission

    for mission_item in mission_waypoints:
        #print("-- Message Read " + str(the_connection.recv_match(type="MISSION_ITEM_REACHED", condition= 'MISSION_ITEM_REACHED.seq == {0}'.format(mission_item.seq) , blocking=True)))
        print("-- Message Read " + str(the_connection.recv_match(type="MISSION_ITEM_REACHED", blocking=True)))
        print("wp>{0}".format(len(mission_waypoints)))
        print("->{0}".format(mission_item.seq))
        #print("-- Message Read " + str(the_connection.recv_match(blocking=True)))
        #ack(the_connection, "MISSION_ITEM_REACHED")
        if(mission_item.seq == (len(mission_waypoints))-2):
            break
    

    # check mode, change to GUIDED, wait for GUIDED
    current_mode = get_drone_mode(the_connection)
    if current_mode == None:
        print("Mode error")
    elif current_mode == 'GUIDED':
        print("Continuing in: ",current_mode)
    else:
        print("Change from ", current_mode," to GUIDED")

        mavutil.mavfile.set_mode_apm(the_connection, 4) # 4 -> GUIDED
        ack(the_connection, "COMMAND_ACK")

    msg_temp = the_connection.recv_match(type='TERRAIN_REPORT', blocking=True)
    lat = msg_temp.lat
    lon = msg_temp.lon
    #terrH = int(msg_temp.terrain_height)
    print("lat: ", lat*10**-7)#, "\tn-> ", (lat+1000)*10**-7)
    print("lon: ", lon*10**-7)#, "\tn-> ", (lon+1000)*10**-7)

    temp_track = 0

    while(get_drone_mode(the_connection) == "GUIDED"):
        msg_temp = the_connection.recv_match(type='TERRAIN_REPORT', blocking=True)
        # lat = msg_temp.lat
        # lon = msg_temp.lon
        terrH = (msg_temp.terrain_height)
        # print("lat: ", lat*10**-7)#, "\tn-> ", (lat+1000)*10**-7)
        # print("lon: ", lon*10**-7)#, "\tn-> ", (lon+1000)*10**-7)
        #print("terrain: ", terrH)
        #do_orbit(the_connection, lat, lon)
        #follow_edit_v2(the_connection, lat+1000, lon+1000)
        #follow_edit_v2(the_connection, 457953710, 159733570)
        #follow_edit_v3(the_connection, 457963710, 159743570, 15)
        #follow_edit_v3(the_connection, lat, lon + 1000, 15+terrH)

        if ((temp_track >= 0) & (temp_track < 4)):
            follow_edit_v3(the_connection, lat - 500, lon + 500, 15+terrH)
            print("1:", temp_track)
        elif ((temp_track >= 4) & (temp_track < 8)):
            follow_edit_v3(the_connection, lat, lon + 1000, 15+terrH)
            print("2:", temp_track)
        elif ((temp_track >= 8) & (temp_track < 12)):
            follow_edit_v3(the_connection, lat + 500, lon + 500, 15+terrH)
            print("3:", temp_track)
        elif ((temp_track >= 12) & (temp_track < 16)):
            follow_edit_v3(the_connection, lat, lon, 15+terrH)
            print("4:", temp_track)
        
        temp_track = temp_track + 1
        if (temp_track > 16):
            temp_track = 0
        
        time.sleep(1)

    mavutil.mavfile.set_mode_rtl(the_connection)
    ack(the_connection, "COMMAND_ACK")

    #the_connection.close()