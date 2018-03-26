cmake_minimum_required(VERSION 2.8.3)
project(conveyor_object_tracking)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
 #list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs pcl_ros cv_bridge message_generation tf2 tf2_ros)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED COMPONENTS core highgui video)

###################################
## catkin specific configuration ##
####################################
#add_message_files(
#    FILES 
#    ConveyorTrajectory.msg
#    )

generate_messages(
    DEPENDENCIES 
    std_msgs 
    geometry_msgs
    )

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES conveyor_object_tracking
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS})

## conveyor scanning node - calculates kinect to conveyor tf
add_executable(conveyor_scanning_node
    src/conveyor_scanning_node.cpp
    src/Conveyor_scanner.cpp
    src/Background_filter.cpp)
target_link_libraries(conveyor_scanning_node ${catkin_LIBRARIES})

## conveyor filtering node - filters original pointcloud data
add_executable(conveyor_filtering_node
    src/conveyor_filtering_node.cpp
    src/Conveyor_filter.cpp)
target_link_libraries(conveyor_filtering_node ${catkin_LIBRARIES})

## object tracking node - does the rest of the magic
add_executable(object_tracking_node
    src/object_tracking_node.cpp
    src/Object_tracker.cpp
    src/Tracked_object.cpp
    src/Conveyor_trajectory.cpp)
target_link_libraries(object_tracking_node ${catkin_LIBRARIES})
add_dependencies(object_tracking_node conveyor_object_tracking_gencpp)
#add_dependencies(object_tracking_node ${PROJECT_NAME}_generate_messages_cpp )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS conveyor_object_tracking conveyor_object_tracking_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
    PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_conveyor_object_tracking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
