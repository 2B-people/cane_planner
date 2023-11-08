cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
  tf
  geometry_msgs
  plan_env
  path_searching
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATIKIN_DEPENDS plan_env
 path_searching
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

# replan FSM
add_executable(kin_replan_node
		src/kin_replan_node.cpp
    src/planner_manager.cpp
)
target_link_libraries(kin_replan_node 
  ${catkin_LIBRARIES}
  )

# sim
add_executable(sim_plan_node
    src/sim_plan_node.cpp
    src/planner_manager.cpp
)
target_link_libraries(sim_plan_node 
  ${catkin_LIBRARIES}
)