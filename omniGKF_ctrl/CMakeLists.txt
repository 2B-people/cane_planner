cmake_minimum_required(VERSION 2.8.3)
project(omniGKF_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  std_srvs
  message_generation
  # three_dof_planar_manipulator
)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  omniGKFcmd.msg
  omniGKFinfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES omniGKF_control
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${SERIAL_INCLUDE_DIRS}
)


# add_executable(gkf_hardware_interface src/gkf_hardware_interface_node.cpp)
#add_dependencies(robot_hardware_interface ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gkf_hardware_interface ${catkin_LIBRARIES})

add_executable(gkf_test_usb src/test_usb.cpp
src/omni_gkf_usb.cpp
)
#add_dependencies(robot_hardware_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(gkf_test_usb ${catkin_LIBRARIES} ${SERIAL_LIBRARIES})


add_executable(gkf_server_node src/omni_gkf_server.cpp
src/omni_gkf_usb.cpp
)

target_link_libraries(gkf_server_node ${catkin_LIBRARIES} ${SERIAL_LIBRARIES})

