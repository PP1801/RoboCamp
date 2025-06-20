import math

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

