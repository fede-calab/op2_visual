cmake_minimum_required(VERSION 3.0.2)
project(op2_visual)

# --------------------------
# C++ Standard Configuration
# --------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------
# Find Catkin Packages
# --------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  ball_msgs
)

# --------------------------
# Hostname-Based Configuration
# --------------------------
execute_process(
    COMMAND hostname
    OUTPUT_VARIABLE HOSTNAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(HOSTNAME STREQUAL "minipc")
    set(OpenCV_DIR /home/minipc/opencv4.5-custom/lib/cmake/opencv4)
endif()

# --------------------------
# Find OpenCV
# --------------------------
find_package(OpenCV 4 REQUIRED)

# --------------------------
# Catkin Package Declaration
# --------------------------
catkin_package(
   CATKIN_DEPENDS roscpp roslib ball_msgs
)

# --------------------------
# Include Directories
# --------------------------
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# --------------------------
# Source Files
# --------------------------
set(OP2_VISUAL_SOURCES
  src/op2_visual_node.cpp
  src/ball_tracking.cpp
  src/ball_detection.cpp
  src/marker_detection.cpp
  src/display_utils.cpp
)

# --------------------------
# Executable Definitions
# --------------------------
add_executable(op2_visual_node ${OP2_VISUAL_SOURCES})

# Specify additional include directories for the executable
target_include_directories(${PROJECT_NAME}_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

# Link Libraries to the Executable
target_link_libraries(op2_visual_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)