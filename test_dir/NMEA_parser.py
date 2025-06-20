#!/usr/bin/env python3

# last updated 06.02.2025.

from decimal import Decimal, getcontext

def parse_gngga(nmea_message, detail=1):
    # Remove the leading '$' and split the message by commas
    parts = nmea_message.decode('ascii').strip().split(',')

    # Extract fields
    time_utc = parts[1]     # UTC time
    lat_raw = parts[2]      # Latitude in NMEA format
    lat_dir = parts[3]      # Latitude hemisphere (N/S)
    lon_raw = parts[4]      # Longitude in NMEA format
    lon_dir = parts[5]      # Longitude hemisphere (E/W)
    fix_quality = parts[6]  # Fix quality
    num_satellites = parts[7]   # Number of satellites
    hdop = parts[8]             # Horizontal dilution of precision
    altitude = parts[9]         # Altitude above mean sea level
    altitude_units = parts[10]  # Altitude units (M)
    geoid_height = parts[11]    # Height of geoid above WGS84 ellipsoid
    geoid_units = parts[12]     # Geoid height units (M)

    # Parse coordinates to decimal degrees
    if lat_raw != "" and lon_raw != "":
        latitude = nmea_to_decimal(lat_raw, lat_dir)
        longitude= nmea_to_decimal(lon_raw, lon_dir)
        altitude = float(altitude)
    else:
        latitude = None
        longitude = None

    if detail == 1:
        # Return parsed data
        return {
            "time_utc": time_utc,
            "latitude": latitude,
            "longitude":longitude,
            "fix_quality":    int(fix_quality),
            "num_satellites": int(num_satellites),
            "hdop":     float(hdop),
            "altitude": float(altitude),
            "geoid_height":float(geoid_height),
        }
    else:
        return {
            "latitude": latitude,
            "longitude":longitude,
            "altitude": altitude,            
        }

def nmea_to_decimal(coord, direction):
    """
    Convert NMEA coordinate format to decimal degrees.
    - Latitude is in ddmm.mmmmm format
    - Longitude is in dddmm.mmmmm format
    """
    
    # Determine degrees and minutes based on the length of the coordinate
    if len(coord.split('.')[0]) > 4:    # Longitude (dddmm.mmmmm)
        # degrees = int(coord[:3])
        # minutes = float(coord[3:])
        degrees = Decimal(coord[:3])
        minutes = Decimal(coord[3:])
    else:                               # Latitude (ddmm.mmmmm)
        # degrees = int(coord[:2])
        # minutes = float(coord[2:])
        degrees = Decimal(coord[:2])
        minutes = Decimal(coord[2:])
    # Convert to decimal degrees
    # decimal_degrees = float(degrees + minutes / 60.0)
    decimal_degrees = degrees + (minutes / Decimal(60))


    # Apply hemisphere correction
    if direction in ['S', 'W']:
        decimal_degrees *= -1

    return decimal_degrees

def check_is_nmea(message):
    try:
        decoded_message = message.decode('ascii').strip()
        #decoded_message = message.decode('utf-8').strip()

        return decoded_message.startswith('$')
    except:
        return 0

# # Example NMEA message
# fake_points = [ b"$GNGGA,091950.25,4547.7617991,N,01558.4370742,E,2,12,0.66,123.341,M,41.375,M,,0000*4F",
#                 b"$GNGGA,091946.75,4547.7618141,N,01558.4371248,E,2,12,0.67,123.252,M,41.375,M,,0000*4B",
#                 b"$GNGGA,091943.25,4547.7618385,N,01558.4372001,E,2,12,0.66,123.135,M,41.375,M,,0000*4E",
#                 b"$GNGGA,091939.75,4547.7618390,N,01558.4372229,E,2,12,0.66,123.009,M,41.375,M,,0000*44"]

# nmea_message = b"$GNGGA,091939.75,4547.7618390,N,01558.4372229,E,2,12,0.66,123.009,M,41.375,M,,0000*44"
# not_nmea_message = b"Some other protocol message"

# message = fake_points[2]

# # Parse the NMEA message
# if check_is_nmea(message):
#     parsed_data = parse_gngga(message)
#     lon, lat = parsed_data['longitude'], parsed_data['latitude']
#     print("Parsed Data:")
#     print(f"lon: {parsed_data['longitude']}")
#     print(f"lat: {parsed_data['latitude']}")
#     print(f"lon: {lon:.7f}")
#     print(f"lat: {lat:.7f}")
#     # for key, value in parsed_data.items():
#     #     print(f"{key}: {value}")
# else:
#     print("Not NMEA:")
#     print(message)