import time
import math, struct
from datetime import datetime
import sys, argparse
from pathlib import Path

VIRTUAL_SERIAL_PORT = "/dev/ttyROBO0"
LOG_FILE_NAME = "pos_log.txt"

cmd_parser = argparse.ArgumentParser(description='Input parameters')
cmd_parser.add_argument('-a', action='store_true', help='Animate markers trough time')
cmd_parser.add_argument('-l', type=int, help='Set limit (int) to the number of displayed markers')
cmd_parser.add_argument('-d', type=float, help='Set delay (float) for animation speed')
cmd_parser.add_argument('-f', type=str, help='Position log filename')
cmd_parser.add_argument('-c', action='store_true', help='Just clear existing markers on the map')
cmd_parser.add_argument('-v', action='store_true', help='.csv conversion')
cmd_args = cmd_parser.parse_args()

def send_planner(_serial_data):
    """Send marker data to Mission Planner via Virtual Serial Port."""
    try:
        with open(VIRTUAL_SERIAL_PORT, 'wb') as port:
            _serial_data = _serial_data.encode('utf-8')
            port.write(_serial_data)
            port.write(b'\n')
    except Exception as e:
        print(e)

animate = False     # [bool] Animation toggle
marker_limit = 600  # [/] Marker limit for drawing on a map
time_delay = 0.5    # [s] Delay between displaying two markers

old_seconds = 0

csv_conv = False

if (cmd_args.a):
    animate = True   
if (cmd_args.l is not None):
    marker_limit = cmd_args.l
    if marker_limit == -1:
        marker_limit = None
    print(f"Set marker limit to: {marker_limit}")
if (cmd_args.d is not None):
    time_delay = float(cmd_args.d)
    print(f"Set animation delay to: {cmd_args.d:.2f}")
if (cmd_args.f is not None):
    LOG_FILE_NAME = cmd_args.f
    if Path(LOG_FILE_NAME).exists():
        print(f"Found {LOG_FILE_NAME}")
    else:
        print(f"{LOG_FILE_NAME} not found!")
        sys.exit()
if (cmd_args.c):
    print("Cleared markers!")
    send_planner(f"off,0")
    sys.exit()
if (cmd_args.v):
    print("Interpret as .csv")
    csv_conv = True

def limit_reached(_index):
    """Return bool depending on the limit breach."""
    if marker_limit is not None:
        if _index <= marker_limit:
            return 0
        else:
            return 1
    else:
        return 0

def parse_plog(_line):
    parts = _line.split(',')

    if len(parts) == 6:
        timestamp = parts[0]
        tmp_lat = parts[1]
        tmp_lon = parts[2]
        fix_qlt = parts[3]
        num_sat = parts[4]
        hdop = parts[5]
        return {
            "time"  :   timestamp,
            "lat"   :   tmp_lat,
            "lon"   :   tmp_lon,
            "fix_q" :   fix_qlt,
            "num_s" :   num_sat,
            "hdop"  :   float(hdop),
        }
    else:
        return 0
    
def parse_csv(_line, type=1):
    if type == 1:
        parts = _line.split(',')
    elif type == 2:
        parts = _line.split(',')

    if len(parts) == 4:
        timestamp = float(int(parts[2])/1000)
        tmp_lat = float(int(parts[0])*(10**-7))
        tmp_lon = float(int(parts[1])*(10**-7))
        fix_qlt = 1 #parts[3]
        num_sat = parts[3]
        hdop = 1 #parts[5]
        return {
            "time"  :   timestamp,
            "lat"   :   tmp_lat,
            "lon"   :   tmp_lon,
            "fix_q" :   fix_qlt,
            "num_s" :   num_sat,
            "hdop"  :   float(hdop),
        }
    elif type == 2:
        timestamp = float(parts[0]) #int(parts[0]/100))
        tmp_lat = float(int(parts[1])*(10**-7))
        tmp_lon = float(int(parts[2])*(10**-7))
        fix_qlt = 1
        num_sat = 10
        hdop = 1
        return {
            "time"  :   timestamp,
            "lat"   :   tmp_lat,
            "lon"   :   tmp_lon,
            "fix_q" :   fix_qlt,
            "num_s" :   num_sat,
            "hdop"  :   float(hdop),
        }
    else:
        return 0


def parse_time(_time):
    parts = _time.split(':')

    if len(parts) == 3:
        hours = parts[0]
        minutes = parts[1]
        seconds = parts[2]
        return {
            "h" : int(hours),
            "m" : int(minutes),
            "s" : int(seconds),
        }
    else:
        return 0

if __name__ == '__main__':
    print(f"Opening {LOG_FILE_NAME}")
    send_planner(f"off,0")  # Clear all markers
    line_count = 0
    if animate:
        with open(LOG_FILE_NAME, "r") as file:
            for line in file:
                line_count += 1
        file.close()
                    # if i_marker % int(line_count/10) == 0:
                    #     print(".", end="", flush=True)
        print(f"animating...{line_count}")
        print(":0.......100:")
        print(":", end="", flush=True)

    with open(LOG_FILE_NAME, "r") as file:
        line = file.readline()
        i_marker = 0
        double_time = False
        #old_seconds = parse_time(parse_plog(line.strip)['time'])['s']
        while line and not limit_reached(i_marker):
            #print(line.strip())

            if not csv_conv:
                log_data = parse_plog(line.strip())
            else:
                log_data = parse_csv(line.strip(), type=2)
                
            if log_data != 0:
                bracelet_ip = log_data['time']
                tmp_lat = log_data['lat']
                tmp_lon = log_data['lon']
                fix_q = log_data['fix_q']
                hdop = log_data['hdop']

                if  hdop <= 1.0: ptype = 1
                elif hdop > 1.0 and hdop <= 1.25:   ptype = 3
                elif hdop > 1.25 and hdop <= 1.5:   ptype = 2
                else:   ptype = 0

                if fix_q == "2":    pstate = 1
                else:               pstate = 0

                serial_data = f"{bracelet_ip},{ptype},{pstate},{tmp_lat},{tmp_lon},0"

                if (animate):
                    new_seconds = parse_time(log_data['time'])['s']

                    if i_marker == 0:
                        old_seconds = new_seconds - 1

                    if (new_seconds == old_seconds + 1): # Expected case
                        #print(new_seconds)
                        send_planner(f"off,0")
                        send_planner(serial_data)
                        time.sleep(time_delay)

                        double_time = False

                    elif (new_seconds == old_seconds):  # Double time stamp
                        #print(f"{new_seconds} +1 (double)")
                        new_seconds += 1
                        double_time = True

                        send_planner(f"off,0")
                        send_planner(serial_data)
                        time.sleep(time_delay)

                    elif (new_seconds >= old_seconds + 2 # Larger time skip
                        and not double_time):
                        for i_ in range(1, (new_seconds-old_seconds)):
                            #print(f"{old_seconds} +{i_} (skipped)")
                            time.sleep(time_delay)
                        #print(f"{new_seconds} (after skip)")
                        send_planner(f"off,0")
                        send_planner(serial_data)
                        time.sleep(time_delay)
                
                    old_seconds = new_seconds
                    if old_seconds == 59:
                        old_seconds = -1
                # if animate:
                #     send_planner(f"off,0")
                #     send_planner(serial_data)
                #     time.sleep(time_delay)
                    if i_marker % int(line_count/10) == 0:
                    #if int(i_marker*100/line_count) % int(10/line_count):
                        print(".", end="", flush=True)

                else:
                    send_planner(serial_data)
            else:
                time.sleep(time_delay)

            line = file.readline()
            i_marker = i_marker + 1
    print(":")
    file.close()
    print("Done!")