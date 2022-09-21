cmake_minimum_required(VERSION 3.0.2)
project(cane_path_searching)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED
		roscpp
		rospy
		std_msgs
		nav_msgs
)

catkin_package(
	  INCLUDE_DIRS include	
		LIBRARIES cane_path_searching
# CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	SYSTEM
	include
	${catkin_INCLUDE_DIRS}
	${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)


add_library(cane_path_searching
	src/Astar_searcher
)

target_link_libraries(cane_path_searching
	${catkin_LIBRARIES} 	
)
