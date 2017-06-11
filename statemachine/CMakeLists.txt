cmake_minimum_required(VERSION 2.8.3)
add_definitions(-std=c++11)
project(statemachine)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp" )

#a dummy_target so qtcreator have access to all files
######################################################################### 
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

#########################################################################


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  tf
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   visualization_msgs
   geometry_msgs
 )



find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
### For referee
# Use Network
set(  QT_USE_QTNETWORK TRUE )
set(  QT_USE_QTXML TRUE )


# Check for QT4
FIND_PACKAGE( Qt4 REQUIRED )
# Load QT variabls
INCLUDE( ${QT_USE_FILE} )

# External header
INCLUDE_DIRECTORIES( ${QT_INCLUDE_DIR} )

find_package(Qt4 COMPONENTS QtNetwork QtXml REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy message_runtime std_msgs std_srvs visualization_msgs geometry_msgs sensor_msgs cv_bridge image_transport image_geometry tf
  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  hermes/inc/
  imageprocessing/inc/
  include/StateMachines/
  ../navigation/move/inc/
  ../navigation/pid/inc/
  ../navigation/vector2d/inc/
  ../localization/amcl/inc/

)

## Declare a C++ library
QT4_WRAP_CPP( SM_CPP include/StateMachines/GlobalStateMachine.h )
add_library(statemachine
src/StateMachines/GlobalStateMachine.cpp src/StateMachines/ShootMachine.cpp src/StateMachines/CalcABMachine.cpp src/StateMachines/FindColorMachine.cpp
src/roboControl.cpp src/ObjectParser.cpp
${SM_CPP} )


QT4_WRAP_CPP( MOC_CPP hermes/inc/hermes.h hermes/inc/referee.h )
add_library( referee STATIC 
hermes/src/hermes.cpp hermes/src/referee.cpp 
hermes/inc/hermescodes.h hermes/inc/hermes.h hermes/inc/referee.h ${MOC_CPP} )

add_library(imageprocessing
            imageprocessing/src/blob.cpp imageprocessing/inc/blob.h
            imageprocessing/src/blobArray.cpp imageprocessing/inc/blobArray.h
            imageprocessing/src/filter.cpp imageprocessing/inc/filter.h
            imageprocessing/src/image_processing.cpp imageprocessing/src/lidar_processing.cpp
            imageprocessing/src/util.cpp imageprocessing/inc/util.h
            imageprocessing/inc/image_processing.h imageprocessing/inc/lidar_processing.h
#Helper - Vector/Line Library
            imageprocessing/inc/Helper/Line.h imageprocessing/inc/Helper/Line.cpp
            imageprocessing/inc/Helper/LineSegment.h imageprocessing/inc/Helper/LineSegment.cpp
            imageprocessing/inc/Helper/Vector2d.h imageprocessing/inc/Helper/Vector2d.cpp
)

add_library(libpid
    ../navigation/pid/inc/PID.h
    ../navigation/pid/src/PID.cpp
)

add_library(libvector2d
    ../navigation/vector2d/inc/vector2d.h
    ../navigation/vector2d/src/vector2d.cpp
)

add_library(libmove
    ../navigation/move/src/move.cpp
    ../navigation/move/inc/move.h
)


add_dependencies(statemachine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/statemachine_node.cpp)
add_executable(restart_node src/restart.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(statemachine_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(restart_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries ( referee
	${QT_LIBRARIES}
)

target_link_libraries ( imageprocessing
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

target_link_libraries(libmove
   libpid
   libvector2d
   ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${QT_LIBRARIES}
   referee
   imageprocessing
   libmove
#   ${aruco_LIBS}
)

target_link_libraries(restart_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}

)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS statemachine statemachine_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_statemachine.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
