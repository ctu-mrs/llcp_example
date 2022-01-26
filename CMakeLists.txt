cmake_minimum_required(VERSION 3.1.2)
project(llcp_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  nodelet
  mrs_msgs
  )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES LlcpExample
  CATKIN_DEPENDS roscpp mrs_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )


add_library(LlcpExample
  src/llcp_example.cpp
)
