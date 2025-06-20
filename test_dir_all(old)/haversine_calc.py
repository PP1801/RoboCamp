#from haversine_distance import haversine_distance
import math

#import matplotlib.pyplot as plt
#from haversine import haversine, inverse_haversine, Direction
#import numpy as np

#import argparse

def haversine_calculator(lat1, lon1, lat2, lon2, radius=2.5):
    """
    Parameters:
    - lat1, lon1: Latitude and Longitude of moving point 1 (drone) in decimal degrees
    - lat2, lon2: Latitude and Longitude of stationary point 2 (bracelet) in decimal degrees

    Returns:
    - lat3, lon3: Latitude and Longitude of target point that is [radius] meters away from actual point 2
    - Distance between the final target and point 2 in meters
    - Angle alpha between final target and current point
    """
    # convert to normal WGS84 (lon 15.97, lat 45.79) from ArduPilot E7
    lat1 = lat1 * 10**-7
    lon1 = lon1 * 10**-7
    lat2 = lat2 * 10**-7
    lon2 = lon2 * 10**-7
    #radius = 2.2458 * 10**-3
    #radius = radius * 0.89832 * 10**-3

    dist = haversine_distance(lat1, lon1, lat2, lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    alpha = math.atan2(dlat, dlon)

    if (dist > radius):

        dist_wgs = (dlon**2 + dlat**2)*((dist-radius)/dist)**2
        delt_lon = math.copysign(1,dlon) * math.sqrt((dist_wgs)/(1 + math.tan(alpha)**2))
        delt_lat = math.tan(alpha) * delt_lon

        lat3 = lat1 + delt_lat  # target point coordinates
        lon3 = lon1 + delt_lon
        #dist_final = haversine_distance(lat3, lon3, lat2, lon2)

    else:
        lat3 = lat1
        lon3 = lon1
        #dist_final = -1

    # also convert coordinates back to ArduPilot E7
    return [lat3*10**7, lon3*10**7, dist, alpha]


def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the distance between two GPS coordinates in meters using the Haversine formula.

    Parameters:
    - lat1, lon1: Latitude and Longitude of point 1 in decimal degrees
    - lat2, lon2: Latitude and Longitude of point 2 in decimal degrees

    Returns:
    - Distance between the two points in meters
    """
    R = 6378137  # Radius of the Earth in meters (WGS84 standard)

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Differences in coordinates
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Haversine formula
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in meters
    distance = R * c
    return distance

# # Example usage

# lat1 = 45.7953420  # Drone's latitude
# lon1 = 15.9730817  # Drone's longitude
# lat2 = 45.7953420 + 10*10**-7 # Beacon's latitude
# lon2 = 15.9730817 + 10*10**-7 # Beacon's longitude

# distance = haversine_distance(lat1, lon1, lat2, lon2)
# print(f"Distance: {distance:.2f} meters")
