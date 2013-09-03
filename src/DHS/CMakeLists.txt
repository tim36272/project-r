cmake_minimum_required(VERSION 2.8.3)
project(dhs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(cv_bridge_DIR "/opt/ros/groovy/stacks/cv_bridge/catkin_generated/installspace")
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp message_generation)

#redirect OpenCV to OpenGL installation
set(OpenCV_DIR "/opt/ros/groovy/stacks/opencv2/share/OpenCV")

#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#add_definitions(${OpenCV_DEFINITIONS})
#MESSAGE(STATUS "OpenCV at: ${OpenCV_INCLUDE_DIRS}")

find_package (OpenCV 2.4.5 REQUIRED)
find_package (cv_bridge REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   person.msg
   bag.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES tim
  CATKIN_DEPENDS cv_bridge image_transport roscpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} "/home/tim/ros_catkin_ws/src/cv_bridge/include/cv_bridge")
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(tim
#   src/${PROJECT_NAME}/tim.cpp
# )

## Declare a cpp executable
# add_executable(tim_node src/tim_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(tim_node tim_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(tim_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS tim tim_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_library(MessageFetcher include/MessageFetcher.cpp)
target_link_libraries(MessageFetcher ${catkin_LIBRARIES})
link_libraries(MessageFetcher)

add_library(Kalman include/Kalman.cpp)
target_link_libraries(Kalman ${catkin_LIBRARIES})
link_libraries(Kalman)

add_library(BlobDescriptor include/BlobDescriptor.cpp)
target_link_libraries(BlobDescriptor ${catkin_LIBRARIES})
link_libraries(BlobDescriptor)


add_library(Analysis include/Analysis.cpp)
target_link_libraries(Analysis ${catkin_LIBRARIES})
link_libraries(Analysis)

add_library(OfflineAnalysis include/OfflineAnalysis.cpp)
target_link_libraries(OfflineAnalysis ${catkin_LIBRARIES})
link_libraries(OfflineAnalysis)

add_executable (live src/live.cpp)
target_link_libraries(live ${catkin_LIBRARIES} ${GTK3_LIBRARIES})

add_executable (picker src/picker.cpp)
target_link_libraries(picker ${catkin_LIBRARIES} ${GTK3_LIBRARIES})

add_executable (offline src/offline.cpp)
target_link_libraries(offline ${catkin_LIBRARIES} ${GTK3_LIBRARIES})

add_executable (projection src/projection.cpp)
target_link_libraries(projection ${catkin_LIBRARIES} ${GTK3_LIBRARIES})

