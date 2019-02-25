cmake_minimum_required(VERSION 2.8.3)
project(FPGA_template)

# TODO set compile option
# set(CMAKE_CXX_FLAGS "-std=c++11 -lm -g -fsigned-char -fpermissive -Wint-to-pointer-cast -O3")



find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  move_base_msgs
  cv_bridge
  roscpp
  std_msgs
  image_transport
  geometry_msgs
)

catkin_package(
)

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)

# Required program
# add_executable(${runtime_name} src/src1.cpp src/src2.cpp etc...)
add_executable(template src/template.cpp)

# Required libraries
target_link_libraries(template ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})