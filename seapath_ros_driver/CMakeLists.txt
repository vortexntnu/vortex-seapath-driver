#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5)
project(seapath_ros_driver)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-local-typedefs -fopenmp)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(tf2              REQUIRED)
find_package(diagnostic_msgs  REQUIRED)


include_directories(include)

add_executable(${PROJECT_NAME}_node
  src/seapath_ros_driver.cpp
  src/seapath_ros_driver_node.cpp
  src/seapath_socket.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  diagnostic_msgs
  vortex_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
  )



 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()


endif()




ament_package()
