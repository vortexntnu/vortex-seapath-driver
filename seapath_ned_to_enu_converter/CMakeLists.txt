cmake_minimum_required(VERSION 3.0.2)
project(seapath_ned_to_enu_converter)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(
    ${PROJECT_NAME}_node
    src/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
