# [Seapath_ros_driver_node]

## Goal
  - Driver for extracting data from the Seapath 13x OEM

  - Receives packets of size 132B containing the KMBinaryData struct over a UDP Socket bound to port 31421.

  - The driver then publishes the data gathered over the ROS2 network in various publishers.

## Publishers of

- [`/diagnostic`]
- [`/odom/origin`]
- [`/seapath/odom/ned`]
- [`/seapath/kmbinary`]
- [`/seapath/navsatfix`]

## Broadcast
- [`/tf`]

## Service
- [`set_origin`]

## Topics

- ### `/diagnostic`

  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`diagnostic_msgs/DiagnosticArray`](https://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html) |
  
  - #### Function
  
    - Provide informatiotion regard status of the hardware connection and the quality of the data it receives
    - In case of error it contain also the error's name and message

- ### `/odom/origin`

  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`sensor_msgs/NavSatFix`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html) |
  
   - #### Function
      - Provide the position of the origin frame in world frame

- ### `/seapath/odom/ned`

  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`nav_msgs/msg/Odometry`](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html) |

  
    - #### Function
  
      - Provides the ASV’s **current position, velocity, and orientation** in the *NED* (North-East-Down) coordinate frame.  
      - This topic is essential for **localization** and **feedback control**, allowing guidance and control nodes to compute reference trajectories and correct deviations from the desired path.  
      - It serves as the **primary source of navigation data** within the autonomous system.


- ### `/seapath/kmbinary`

  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`Custom`](https://github.com/vortexntnu/vortex-msgs/blob/main/msg/KMBinary.msg) |
  
  - #### Function
  
    - It provide the raw data from GNSS


- ### `/seapath/navsatfix`

  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`sensor_msgs/NavSatFix`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html) |
  
   - #### Function
  
      - Provide the current position in the world frame, in particular returns coordinates in latitude and longitude
  
## Broadcast
- ### `/tf`

  - #### message
    - Trasformation matrix from the world frame to the vehicle frame, computed using a quaternion

## Service

- ### `set_origin`

  - #### Service type
    - This service is a service-trigger: it don't need input and as output it give a boolean value to ensure success or failure of the service and mesagge with more details

  - #### Function
    - Save current position as frame origin
