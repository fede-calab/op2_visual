cmake_minimum_required(VERSION 3.0.2)
project(op2_visual)



# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

# Optionally, you can enforce the standard (no other version can be used)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optionally, you can specify whether C++17 features are enabled even with older compilers
set(CMAKE_CXX_EXTENSIONS OFF)  # This ensures you're using pure C++17 without compiler-specific extensions



## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ball_msgs
)

execute_process(
    COMMAND hostname
    OUTPUT_VARIABLE HOSTNAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(HOSTNAME STREQUAL "minipc")
    set(OpenCV_DIR /home/minipc/opencv4.5-custom/lib/cmake/opencv4)
endif()

find_package(OpenCV 4 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES op2_visual
   CATKIN_DEPENDS roscpp ball_msgs

)

###########
## Build ##
###########



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)

set(SOURCES
    src/op2_visual_node.cpp
    # src/Aruco_det.cpp
    src/ball_tracking.cpp
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/op2_visual.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node 
#     ${SOURCES}
# )

add_executable(op2_visual_node
  src/op2_visual_node.cpp
  src/ball_tracking.cpp
  src/ball_detection.cpp
  src/marker_detection.cpp
  src/display_utils.cpp
  src/shared_memory_utils.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME}_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/aruco-3.1.12/src
    ${OpenCV_INCLUDE_DIRS}
    pthread
    rt
)

## Specify libraries to link a library or executable target against
target_link_libraries(op2_visual_node
    # aruco
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pthread
    rt
)

## Calibration Executable
# set(SOURCES_CALIBRATION
#     src/op2_visual_calibration_node.cpp
#     src/Aruco_det.cpp
#     src/ball_tracking.cpp
# )

# add_executable(${PROJECT_NAME}_calibration_node ${SOURCES_CALIBRATION})

# target_include_directories(${PROJECT_NAME}_calibration_node PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/vendor/aruco-3.1.12/src
#     ${OpenCV_INCLUDE_DIRS}
# )

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_calibration_node
#     aruco
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_op2_visual.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#Codice suggerito
# Add the executable for the test node
# add_executable(webcam_test_node src/webcam_test_node.cpp)

# Link necessary libraries
# target_link_libraries(webcam_test_node
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )

## Declare the executable
# add_executable(camera_calibration_node src/camera_calibration_node.cpp)

## Specify libraries to link against
# target_link_libraries(camera_calibration_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

## Declare the executable
# add_executable(compute_distance_between_aruco_node src/compute_distance_between_aruco.cpp)

## Specify libraries to link against
# target_link_libraries(compute_distance_between_aruco_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

## Declare the executable
# add_executable(compute_distance_between_aruco_img_node src/compute_distance_between_aruco_img.cpp)

## Specify libraries to link against
# target_link_libraries(compute_distance_between_aruco_img_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

## Declare the executable
# add_executable(compute_position_node src/compute_position_of_ball_between_markers.cpp)

## Specify libraries to link against
# target_link_libraries(compute_position_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )


## Declare the executable
# add_executable(ball_position_webcam_node src/ball_position_webcam.cpp)
# target_link_libraries(ball_position_webcam_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   pthread
#   rt
# )

## Declare the executable
add_executable(offset_management_node src/offset_management.cpp)
target_link_libraries(offset_management_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pthread
  rt
)

install(TARGETS op2_visual_node offset_management_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)