# vortex-seapath-driver
Driver for extracting pose data from the Seapath 13x OEM

Receives packets of size 132B containing the KMBinaryData struct over a UDP Socket bound to port 31421.

The driver then publishes the data gathered over the ROS2 network in various publishers.