cmake_minimum_required(VERSION 3.0.2)
project(revoMotor)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CATKIN_PACKAGE_DEPENDENCIES
  message_generation
  roscpp
  rospy
  serial
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  roslib
)


## Generate messages in the 'msg' folder
 add_message_files(
    FILES
    statecmd.msg
    motorinfo.msg
    motorcmd.msg

 )

 add_service_files(
  FILES
  # motormode.srv
  # resetmode.srv
)
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS 
   include
  #LIBRARIES revoMotor
  CATKIN_DEPENDS 
   ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_info
	include/revoMotor/revodriver.h
  src/driver/revodriver.cpp
  )
  add_library(${PROJECT_NAME}_test_info
	include/revoMotor/revodriver.h
  src/driver/revodriver_test.cpp
  )

add_executable(revodriver_node 
		src/driver/revodriver.cpp)
		
add_dependencies(revodriver_node
	${catkin_EXPORTED_TARGETS})
	
target_link_libraries(revodriver_node
  ${PROJECT_NAME}_info
  ${catkin_LIBRARIES})

  add_executable(revodriver_test_node 
		src/driver/revodriver_test.cpp)
		
add_dependencies(revodriver_test_node
	${catkin_EXPORTED_TARGETS})
	
target_link_libraries(revodriver_test_node
  ${PROJECT_NAME}_test_info
  ${catkin_LIBRARIES})


# add_executable(motormode src/driver/motormode_server.cpp)
# target_link_libraries(motormode ${catkin_LIBRARIES})
# add_dependencies(motormode ${catkin_EXPORTED_TARGETS})

# add_executable(resetmode src/driver/resetmode_server.cpp)
# target_link_libraries(resetmode ${catkin_LIBRARIES})
# add_dependencies(resetmode ${catkin_EXPORTED_TARGETS})



