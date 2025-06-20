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

target_altitude = 15    # altitude for takeoff

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
    ack(the_connection, "COMMAND_ACK")

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
        print("No heartbead received.")
        return None

    base_mode = msg.base_mode
    armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    armable = ((base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0) or ((base_mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED) != 0)
    return [armed, armable]

# Get current mode of the Drone
def get_drone_mode(the_connection):
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)    #todo: check timeout
    if not msg:
        print("No heartbead received.")
        return None
    
    custom_mode = msg.custom_mode
    #base_mode = msg.base_mode
    mode = mavutil.mode_mapping_acm.get(custom_mode)
    return mode

# Send message for the drone to return to the launch point
def set_return(the_connection):
    print("-- Set RTL")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        20, #mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,0,0,0,0,0,0,0
    )
    ack(the_connection, "COMMAND_ACK")

# Start Mission
def start_mission(the_connection):
    print("-- Mission Start")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        300, #mavutil.mavlink.MAV_CMD_MISSION_START,
        0,0,0,0,0,0,0,0
    )
    ack(the_connection, "COMMAND_ACK")

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


# Orbit the point
def do_orbit(the_connection, lat, lon):
    #print("-- Orbit Start around:")

    #the_connection.mav.command_long_send(
    #    the_connection.target_system,
    #    the_connection.target_component,
    #    144, #mavutil.mavlink.FOLLOW_TARGET ??
    #
    #)

    # MAC_CMD_DO_ORBI (34)
    (
    #the_connection.mav.command_int_send(
    #    the_connection.target_system,
    #    the_connection.target_component,
    #    #the_connection.target_component,    #probaj circle 107
    #    34, #mavutil.mavlink.MAV_CMD_DO_ORBIT # WIP function!!
    #    3,          # Circle radius [m]
    #    2,   # Vehicle velocity [m/s]
    #    1, #ORBIT_YAW_BEHAVIOUR -> ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER
    #    100,#6*math.pi,  # How many orbits [rad]
    #    0,0,0,
    #    lat,   # Latitude center point -> current
    #    lon,   # Longitude center point -> current
    #    15          # Altitude center point [m]
    #)
    )

    # MAV_CMD_NAV_FOLLOW (25)
    ( # unsupported
    #the_connection.mav.command_long_send(
    #    the_connection.target_system,
    #    the_connection.target_component,
    #    25, #mavutil.mavlink.MAV_CMD_NAV_FOLLOW, # NOT SUPPORTED BY AP
    #    0,  #
    #    1,  # Following logic
    #    0,  # Ground speed of the vehicle to be followed
    #    3,  # Radius around waypoint
    #    90, # Desured yaw angle
    #    lat,# Latitude
    #    lon,# Longitude
    #    10  # Altitude
    #)
    )

    # MAV_CMD_DO_ORBIT (34) [WIP]
    ( # unsupported
    #the_connection.mav.command_long_send(
    #    the_connection.target_system,
    #    the_connection.target_component,
    #    34, #mavutil.mavlink.MAV_CMD_DO_ORBIT, 
    #    0,  # confirmation
    #    3,          # Circle radius [m]
    #    math.nan,   # Vehicle velocity [m/s]
    #    1, #ORBIT_YAW_BEHAVIOUR -> ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER
    #    100,#6*math.pi,  # How many orbits [rad]
    #    lat,   # Latitude center point -> current
    #    lon,   # Longitude center point -> current
    #    15     # Altitude center point [m]
    #)
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

    #cor = 3
    #mission_waypoints.append(mission_item(0, 0, 45.7953420, 15.9730817, 15)) # Above takeoff point
    #mission_waypoints.append(mission_item(0, 0, 45.7952364, 15.9730643, 10)) # Above destination point
    #mission_waypoints.append(mission_item(1, 0, 45.7952364, 15.9730643, 15))  # Destination point
    mission_waypoints.append(mission_item(0, 0, 44.5375718, 14.9151876, 15)) # Above takeoff point
    mission_waypoints.append(mission_item(1, 0, 44.5367107, 14.9136158, 20))

    mission_waypoints.append(mission_item(2, 0, 44.5360958, 14.9134581, 20))

    mission_waypoints.append(mission_item(3, 0, 44.5361946, 14.9145955, 20))

    mission_waypoints.append(mission_item(4, 0, 44.5369705, 14.9157095, 20))
    mission_waypoints.append(mission_item(5, 0, 44.5372186, 14.9164261, 15))
    mission_waypoints.append(mission_item(6, 0, 44.5378649, 14.9157884, 15))
    
    #mission_waypoints.append(mission_item(4-cor, 0, 45.795398, 15.973256, 15))  # Ograda ulaz
    #mission_waypoints.append(mission_item(5-cor, 0, 45.795371, 15.973357, 15))  # Miramarska ulaz
    #mission_waypoints.append(mission_item(6-cor, 0, 45.796084, 15.973836, 15))  # Miramarska parking
    #mission_waypoints.append(mission_item(7-cor, 0, 45.796185, 15.974452, 15))  # Parking prilaz
    #mission_waypoints.append(mission_item(8-cor, 0, 45.796378, 15.974511, 15))  # Parking sredina

    #44.5367107, 14.9136158
    #44.5360958, 14.9134581
    #44.5361946, 14.9145955
    #44.5369705, 14.9157095
    #44.5372186, 14.9164261
    #44.5378649, 14.9157884


    # check mode, change to GUIDED, wait for GUIDED
    current_mode = get_drone_mode(the_connection)
    if current_mode == None:
        print("Mode error")
    elif current_mode == 'GUIDED':
        print("Continuing in: ",current_mode)
    else:
        print("Change from ", current_mode," to GUIDED")
        mode_guided(the_connection)


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
        temp_waypoints.append(mission_item(0, 0, msg_temp.lat*(10**-7), msg_temp.lon*(10**-7), 15))
        temp_waypoints.append(mission_item(1, 0, msg_temp.lat*(10**-7), msg_temp.lon*(10**-7), 15))
        
        upload_mission(the_connection, temp_waypoints)

        start_mission(the_connection)

    elif state[1]:
        print("Armable")
        arm(the_connection)

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
  
    start_mission(the_connection)

    #while True:
    #    print("-- Msg= " + str(the_connection.recv_msg()))

    for mission_item in mission_waypoints:
        #print("-- Message Read " + str(the_connection.recv_match(type="MISSION_ITEM_REACHED", condition= 'MISSION_ITEM_REACHED.seq == {0}'.format(mission_item.seq) , blocking=True)))
        print("-- Message Read " + str(the_connection.recv_match(type="MISSION_ITEM_REACHED", blocking=True)))
        print("wp>{0}".format(len(mission_waypoints)))
        print("->{0}".format(mission_item.seq))
        #print("-- Message Read " + str(the_connection.recv_match(blocking=True)))
        #ack(the_connection, "MISSION_ITEM_REACHED")
        if(mission_item.seq == (len(mission_waypoints))-2):
            #print("Going out..")
            break
        #if(mission_item == None):
        #    print("mission item is None")
        #    break
    
    print("I'm out..")
    #do_orbit(the_connection)

    #time.sleep(2)

    # check mode, change to GUIDED, wait for GUIDED
    current_mode = get_drone_mode(the_connection)
    if current_mode == None:
        print("Mode error")
    elif current_mode == 'GUIDED':
        print("Continuing in: ",current_mode)
    else:
        print("Change from ", current_mode," to GUIDED")
        mode_guided(the_connection)

    # start circle

    #msg_temp = the_connection.recv_match(type='TERRAIN_REPORT', blocking=True)
    #lat = msg_temp.lat
    #lon = msg_temp.lon
    #print("lat: ", lat*10**-7)
    #print("lon: ", lon*10**-7)

    #set_circle(the_connection, lat, lon)

    #do_orbit(the_connection, lat, lon)
    
    #follow_edit(the_connection, lat + 100, lon + 100)

    # MAV_CMD_DO_FOLLOW (32)
    #the_connection.mav.command_long_send(
    #    the_connection.target_system,
    #    the_connection.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_FOLLOW, #32
    #    0,
    #    255, #the_connection.target_system + 1,
    #    0,0,0,0,0,
    #    20
    #)

    #print("-- Changing Mode to FOLLOW")
    #the_connection.mav.command_long_send(
    #    the_connection.target_system,
    #    the_connection.target_component,
    #    176, #mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    #    0,
    #    1,
    #    23, #mode_mapping_acm: FOLLOW
    #    0,0,0,0,0
    #)


    #ack(the_connection, "COMMAND_ACK")

    throttle_override(the_connection, 1600)
    mode_circle(the_connection)
    throttle_override(the_connection, 1600)
 
    while(get_drone_mode(the_connection) == "CIRCLE"):
        the_connection.recv_match(type='HEARTBEAT', blocking=True)
        throttle_override(the_connection, 1600)
    #    #the_connection.recv_match(type=''))

    # while(get_drone_mode(the_connection) == "GUIDED"):
    #     msg_temp = the_connection.recv_match(type='TERRAIN_REPORT', blocking=True)
    #     lat = msg_temp.lat
    #     lon = msg_temp.lon
    #     print("lat: ", lat*10**-7)#, "\tn-> ", (lat+1000)*10**-7)
    #     print("lon: ", lon*10**-7)#, "\tn-> ", (lon+1000)*10**-7)

        #do_orbit(the_connection, lat, lon)
        
        #the_connection.mav.command_long_send(
        #    the_connection.target_system,
        #    the_connection.target_component,
        #    mavutil.mavlink.MAV_CMD_DO_FOLLOW,
        #    0,
        #    1, #the_connection.target_system + 1,
        #    0,0,0,0,0,
        #    20
        #)        
              

        #the_connection.recv_match(type=''))

 #   set_return(the_connection)

    #the_connection.close()