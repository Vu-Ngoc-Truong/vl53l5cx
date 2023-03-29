cmake_minimum_required(VERSION 2.8.3)
project(vl53mkac)

# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(VL53MKAC src/VL53MKAC.cpp src/lim.cpp)
target_link_libraries(VL53MKAC ${console_bridge_LIBRARIES})


# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(VL53MKAC_ros_node src/VL53MKAC_ros.cpp)
target_link_libraries(VL53MKAC_ros_node VL53MKAC ${catkin_LIBRARIES})


install(TARGETS VL53MKAC VL53MKAC_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
