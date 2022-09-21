cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(cane_planner_node
		src/cane_planner_node.cpp
    src/planner_manager.cpp
)

target_link_libraries(cane_planner_node 
  ${catkin_LIBRARIES}
  )