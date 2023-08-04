#!/usr/bin/python3
import socket

from dataclasses import dataclass
import re

@dataclass
class SensorData:
    roll: float
    pitch: float
    yaw: float
    angular_rate_roll: float
    angular_rate_pitch: float
    angular_rate_yaw: float
    lin_acc_roll: float
    lin_acc_pitch: float
    lin_acc_yaw: float

def parse_nmea_to_dataclass(nmea_string):
    # remove leading/trailing whitespaces, and split by comma
    parts = nmea_string.strip().split(',')
    if len(parts) < 12:
        raise ValueError('Invalid NMEA string')

    # Now parse and assign each part to our dataclass
    sensor_data = SensorData(
        roll=float(parts[3]),
        pitch=float(parts[4]),
        yaw=float(parts[5]),
        angular_rate_roll=float(parts[6]),
        angular_rate_pitch=float(parts[7]),
        angular_rate_yaw=float(parts[8]),
        lin_acc_roll=float(parts[9]),
        lin_acc_pitch=float(parts[10]),
        lin_acc_yaw=float(parts[11].split('*')[0]))  # We split by '*' to get rid of the checksum

    return sensor_data


UDP_IP = "0.0.0.0"
UDP_PORT = 7552

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
print(f"Binding to {UDP_IP}:{UDP_PORT}...")
sock.bind((UDP_IP, UDP_PORT))
print("Done!")

while True:
    data, addr = sock.recvfrom(1024)
    nmea_string = data.decode('utf-8')  # convert bytes to string
    sensor_data = parse_nmea_to_dataclass(nmea_string)
    print(sensor_data)