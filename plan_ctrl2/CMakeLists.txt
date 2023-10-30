cmake_minimum_required(VERSION 2.8.3)
project(gkf_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  rospy
  serial
  std_msgs
  std_srvs
  # three_dof_planar_manipulator
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES gkf_control
  CATKIN_DEPENDS controller_manager roscpp rospy std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(single_joint_hardware_interface src/gkf_hardware_interface_node.cpp)
#add_dependencies(robot_hardware_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_joint_hardware_interface ${catkin_LIBRARIES})
