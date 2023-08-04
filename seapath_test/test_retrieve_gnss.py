#!/usr/bin/python3
import socket

import numpy as np

from dataclasses import dataclass

ORIGIN_N = None
ORIGIN_E = None

@dataclass
class SeapathData:
    north: float = None
    north_sigma: float = None

    east: float = None
    east_sigma: float = None

    altitude: float = None
    altitude_sigma: float = None

    heading: float = None

    sog_mps: float = None
    cog_rad: float = None

    rot: float = None

def displacement_wgs84(north, east):
    # Approximate radius of earth in km, at around 45 degrees latitude
    R = 6371.0

    # Distance per degree latitude in meters
    m_per_deg_lat = R * 1000 * (np.pi / 180.0)

    # Distance per degree longitude at this latitude in meters
    m_per_deg_lon = m_per_deg_lat * np.cos(np.radians(ORIGIN_N))

    # Displacement calculation
    displacement_north = (north - ORIGIN_N) * m_per_deg_lat
    displacement_east = (east - ORIGIN_E) * m_per_deg_lon

    return displacement_north, displacement_east

def convert_dms_to_dd(dms):
    degrees = int(dms / 100)
    minutes = dms - (degrees * 100)
    dd = degrees + (minutes / 60)
    return round(dd, 6)  # limiting the precision to 6 decimal places


def parse_nmea_data(nmea_data):

    global ORIGIN_N, ORIGIN_E

    seapath_data = SeapathData()

    sentences = nmea_data.decode().strip().split('\n')

    # Process each sentence
    for sentence in sentences:

        data, checksum = sentence[1:].split('*')

        parts = data.split(',')
        # Check the sentence type and print the relevant parts
        if parts[0] == 'INGGA': # GNSS fix
            north = parts[2]
            east = parts[4]
            altitude = parts[9]

            if north == "" or east == "":
                continue

            seapath_data.north = convert_dms_to_dd(float(north))
            seapath_data.east = convert_dms_to_dd(float(east))
            seapath_data.altitude = float(altitude)

            if ORIGIN_N is None:
                ORIGIN_N = seapath_data.north
                ORIGIN_E = seapath_data.east

            n, e = displacement_wgs84(seapath_data.north, seapath_data.east)
            #print(f"N [m]: {n} \t E [m]: {e}")

        if parts[0] == 'INHDT': # True heading
            true_north_heading = parts[1]
            if true_north_heading == "":
                continue

            seapath_data.heading = float(true_north_heading)

        if parts[0] == "INVTG": # Course over ground and speed over ground


            ground_speed_mps = parts[7]
            ground_course_rad = parts[1]

            if ground_speed_mps == "" or ground_course_rad == "":
                continue

            seapath_data.sog_mps = np.round(float(ground_speed_mps)*1000.0/3600.0, 6)
            seapath_data.cog_rad = np.round(float(ground_course_rad)* 2 * np.pi / 360.0, 6)


        if parts[0] == "GNGST": #GNSS pseudorange error statistics
            utc_time = parts[1] 
            sigma_fix = parts[2] 
            sigma_semimajor = parts[3] # Standard deviation of semi-minor axis of error ellipse
            sigma_semiminor = parts[4] # Standard deviation of semi-major axis of error ellipse
            error_ellipse_orientation = parts[5] # True north degrees

            sigma_lat = parts[6] 
            sigma_lon = parts[7] 
            sigma_alt = parts[8] 

            if sigma_lat == "" or sigma_lon == "" or sigma_alt == "":
                continue

            seapath_data.north_sigma = float(sigma_lat)
            seapath_data.east_sigma = float(sigma_lon)
            seapath_data.altitude_sigma = float(sigma_alt)

        if parts[0] == "INROT": # Rate of turn
            rate_of_turn = parts[1]

            if rate_of_turn == "":
                continue
            
            seapath_data.rot = rate_of_turn
            

    return seapath_data

UDP_IP = "0.0.0.0"
UDP_PORT = 31420

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
print(f"Binding to {UDP_IP}:{UDP_PORT}...")
sock.bind((UDP_IP, UDP_PORT))
print("Done!")

while True:
    data, addr = sock.recvfrom(1024)
    seapath_data = parse_nmea_data(data)
    print(seapath_data)