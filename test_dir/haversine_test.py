from old_scripts.haversine_distance import haversine_distance
import math

import matplotlib.pyplot as plt
from haversine import haversine, inverse_haversine, Direction
import numpy as np

import argparse

cmd_parser = argparse.ArgumentParser(description='Input parameters.')
cmd_parser.add_argument('-r', action='store_true', help='print debug')
cmd_parser.add_argument('-l', action='store_true', help='plot debug')

cmd_args = cmd_parser.parse_args()

bool_print = False
bool_plot = False

if cmd_args.r is not None:
    print("Print ON")
    bool_print = True
if cmd_args.l is not None:
    print("Plot ON")
    bool_plot = True

# # Define the coordinates (latitude, longitude) in degrees
# lat1, lon1 = 47.397742, 8.545594  # Point 1
# lat2, lon2 = 47.398000, 8.546000  # Center of the circle (Point 2)
# lat3, lon3 = 47.397500, 8.545200  # Point 3

#radius = 1000*(10**-7)  # Circle radius in meters
#radius = 2.2457882103310567 * 10**-3
radius = 2.2458 * 10**-3

lat1 = 45.7953420  # Drone's latitude
lon1 = 15.9730817  # Drone's longitude

lat2 = 45.7953420 - 300*10**-7 # Beacon's latitude
lon2 = 15.9730817 + 1000*10**-7 # Beacon's longitude

#lat2 = 45.795373577784574
#lon2 = 15.973113277784572

dist = haversine_distance(lat1, lon1, lat2, lon2)
#print("dist: ", dist)
if (dist > 2.5):

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    alpha = math.atan2(dlat, dlon)
    dist_wgs = (dlon**2 + dlat**2)*((dist-2.5)/dist)**2
    delt_lon = math.copysign(1,dlon) * math.sqrt((dist_wgs)/(1 + math.tan(alpha)**2))
    delt_lat = math.tan(alpha) * delt_lon

    lat3 = lat1 + delt_lat
    lon3 = lon1 + delt_lon
    dist_final = haversine_distance(lat3, lon3, lat2, lon2)

    if bool_print:
        print("alpha: ", alpha)
        print("dist_wgs: ", dist_wgs)
        print("delt_lat: ", delt_lat)
        print("delt_lon: ", delt_lon)
        print("lat3: ", lat3)
        print("lon3: ", lon3)

        print(f"dist_final: {dist_final:.2f}")

    if bool_plot:
        # Plot the points
        plt.figure(figsize=(8, 8))
        plt.scatter(lon1, lat1, color="green", label="Point 1 - Drone position")
        plt.scatter(lon2, lat2, color="red", label="Point 2 - Person position")
        plt.scatter(lon3, lat3, color="blue",label="Point 3 - Stop point")

        # Draw the circle around (lat2, lon2) with a radius of 100m
        angles = np.linspace(0, 2 * np.pi, 100)  # 100 points for the circle
        circle_coords = [inverse_haversine((lat2, lon2), radius, angle) for angle in angles]
        circle_lats, circle_lons = zip(*circle_coords)
        plt.plot(circle_lons, circle_lats, color="blue", label="Circle (2,5m radius)")

        # Formatting
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title("Points and Circle on Map")
        plt.legend()
        plt.grid()
        plt.axis("equal")  # Keep the aspect ratio equal for proper visualization
        plt.show()

distance = haversine_distance(lat1, lon1, lat2, lon2)
print(f"Distance: {distance:.2f} meters")