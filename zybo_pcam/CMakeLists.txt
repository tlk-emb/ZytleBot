cmake_minimum_required(VERSION 2.8.3)
project(zybo_pcam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS image_transport roscpp std_msgs sensor_msgs)


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

# include_directories(include
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/zybo_pcam.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
