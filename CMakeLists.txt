cmake_minimum_required(VERSION 3.2.0 FATAL_ERROR)

project(rosaria)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS message_generation roscpp nav_msgs geometry_msgs sensor_msgs tf
  dynamic_reconfigure)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_compile_options(-std=c++11)

#uncomment if you have defined messages
add_message_files(
  FILES
  BumperState.msg
  #PathName.msg
  #Measurements.msg
)
#uncomment if you have defined services
add_service_files(
  FILES
  GetWayPoints.srv
)

generate_dynamic_reconfigure_options(cfg/RosAria.cfg)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
include_directories(include)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.cpp"
  PATTERN ".svn" EXCLUDE
 )
#set_property(SOURCE ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/PathFinder.h PROPERTY  CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set_property(SOURCE ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/PathFinder.h PROPERTY  CXX_STANDARD_REQUIRED ON)

find_package(Aria QUIET)
if(Aria_FOUND)
	if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
	# The installation package provided by Adept doesn't follow Debian policies
	if(EXISTS "/usr/local/Aria/include/Aria.h")
	  add_definitions(-DADEPT_PKG)
	  include_directories( /usr/local/Aria/include)
	  link_directories(/usr/local/Aria/lib)
	endif()
endif()

add_executable(RosAria RosAria.cpp LaserPublisher.cpp)
add_executable(interface src/Interface.cpp)
add_executable(print_state print_state.cpp)
add_executable(teleop src/teleop.cpp)
add_executable(testpub src/testpub.cpp)
add_executable(rssimeasure src/rssimeasuretest.cpp)
add_dependencies(RosAria rosaria_gencfg)
add_dependencies(RosAria rosaria_gencpp)

target_link_libraries(RosAria ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)
target_link_libraries(interface 
   ${catkin_LIBRARIES}
)
target_link_libraries(teleop 
   ${catkin_LIBRARIES}
)
target_link_libraries(print_state 
   ${catkin_LIBRARIES}
)

target_link_libraries(testpub
   ${catkin_LIBRARIES}
)

target_link_libraries(rssimeasure
  ${catkin_LIBRARIES}
)

set_target_properties(RosAria PROPERTIES COMPILE_FLAGS "-fPIC")
#set_target_properties(RosAria PROPERTIES LINK_FLAGS "-Wl")

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)
# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs tf
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS 
    RosAria
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)