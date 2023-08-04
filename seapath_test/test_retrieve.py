#!/usr/bin/python3
import socket

def compute_checksum(sentence):
    # Remove the $ at the start and split the sentence into data and checksum
    data, checksum = sentence[1:].split('*')
    
    # Compute the checksum by XORing all characters in the data
    computed_checksum = 0
    for char in data:
        computed_checksum ^= ord(char)
    
    # Convert the computed checksum to hexadecimal and uppercase it
    computed_checksum = format(computed_checksum, '02X')
    
    return data, checksum, computed_checksum

def parse_nmea_data(data):
    # Decode the bytes to a string, remove leading and trailing whitespaces, 
    # and split the data into lines using the '\n' newline character
    sentences = data.decode().strip().split('\n')
    
    # Process each sentence
    for sentence in sentences:
        # Compute the checksum and compare it to the provided one
        data, checksum, computed_checksum = compute_checksum(sentence)
        # if checksum != computed_checksum:
        #     #print(f'Checksum is incorrect! Provided: {checksum}, Computed: {computed_checksum}')
        #     print("incorrect checksum")
        #     continue  # Skip this sentence if the checksum is incorrect
        
        # Split the data into parts using the ',' character
        parts = data.split(',')
        # Check the sentence type and print the relevant parts
        if parts[0] == 'INGGA':
            north = parts[2]
            east = parts[4]
        elif parts[0] == 'INHDT':
            true_north_heading = parts[1]
        print()

UDP_IP = "0.0.0.0"
UDP_PORT = 31420

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
print(f"Binding to {UDP_IP}:{UDP_PORT}...")
sock.bind((UDP_IP, UDP_PORT))
print("Done!")

while True:
    data, addr = sock.recvfrom(1024)
    parse_nmea_data(data)