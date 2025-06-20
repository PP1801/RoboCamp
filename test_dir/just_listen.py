#!/usr/bin/env python3

import math, time
from pymavlink import mavutil

import sys, argparse, getopt
# list of command line arguments
cmd_parser = argparse.ArgumentParser(description='Input parameters.')
cmd_parser.add_argument('-c', type=str, help='filter command')
cmd_parser.add_argument('-u', type=int, help='direct udp port')
cmd_parser.add_argument('-s', type=str, help='connection string')

cmd_args = cmd_parser.parse_args()

#'udp:127.0.0.1:14551'
udp_string = "udp:127.0.0.1:"
filt_cmd = None
new_udp = 14551
connection = None

if (cmd_args.c is not None):    # specific command filter
    filt_cmd = cmd_args.c
    print(("Tracking: %s") % (filt_cmd))

if(cmd_args.s is not None):     # direct udp path input
    connection = str(cmd_args.s)

elif (cmd_args.u is not None):  # just udp port input
    new_udp = cmd_args.u
    print(("On UDP port: %s" % (new_udp)))
    connection = udp_string + str(new_udp)
else:
    connection = udp_string + str(new_udp)

print(("Connection string is: %s") % (connection))


(
#argumentList = sys.argv[1:]
#
#options = "c:"              # Options
#long_options = ["CMD="]     # Long options
#filt_cmd = []           # Filtered command
#
#try:
#    # Parsing argument
#    arguments, values = getopt.getopt(argumentList, options, long_options)
#    
#    for currentArgument, currentValue in arguments:
#
#        if currentArgument in ("-c", "--CMD"):
#            print (("Filtering CMD: %s") % (currentValue))
#            filt_cmd = currentValue
#            
#except getopt.error as err:
#    # output error, and return with an error code
#    print (str(err))
)
#if (filt_cmd != []):
#    print("--Yes: ", filt_cmd)



# Acknoledgement from the Drone 
def ack(the_connection, keyword):
    print("-- Message Read " + str(the_connection.recv_match(type=keyword, blocking=True)))

# Main function
if __name__ == "__main__":
    print("-- Program Started")
    #the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
    #the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14551')

    the_connection = mavutil.mavlink_connection(connection)
    #the_connection = mavutil.mavlink_connection("/dev/ttyUSB0")
    

    while (the_connection.target_system == 0):
        print("-- Checking Heartbeat")
        the_connection.wait_heartbeat()
        print("-- Heartbeat from system (system %u component %u)" 
              % (the_connection.target_system, the_connection.target_component))
        
    while(True):
        if(the_connection.recv_msg() != None):
            if(filt_cmd != []):
                msg = the_connection.recv_match(type=filt_cmd, blocking=True)
                #print("-- Message Read " + str(the_connection.recv_match(type=filt_cmd, blocking=True)))
                #if filt_cmd == "RC_CHANNELS":
                    #print(the_connection.recv_msg())
            else:
                msg = the_connection.recv_match(blocking=True)
                #print("-- Message Read " + str(the_connection.recv_match(blocking=True)))
            #print("-- Message Read " + str(the_connection.recv_match(type="ATTITUDE", blocking=True)))
            print("--Message Read " + str(msg))
            # print("Chan8 = " + str(msg.chan8_raw))

            # if (msg.chan8_raw >= 1000):
            #     print("vece od 1000")
            # if (msg.chan8_raw >= 2000):
            #     print("vece od 2000")
