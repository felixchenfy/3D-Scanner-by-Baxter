cmake_minimum_required(VERSION 2.8.3)
project(scan3d-by-baxter)

# set ccache
find_program(CCACHE_FOUND ccache) 
if(CCACHE_FOUND) 
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache) 
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) 
endif(CCACHE_FOUND) 
set (CMAKE_CXX_FLAGS "-DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS") 

# compiler
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# Method 1: (Need to call "project(xxx)" before this line)
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin ) 
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# Method 2:
# set( EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin )
# set( LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib )

# Method 3:
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

# pcl 
find_package( PCL REQUIRED ) 
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Eigen
include_directories( "/usr/include/eigen3" ) # include eigen. Please modify this to the path on your computer

# OpenCV
find_package( OpenCV 4.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

############### dependencies ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
# add_subdirectory( tutorials ) # please comment out this line

############### ROS ######################
# Set ros package for catkin_make
# Please comment out the lines below if not using ros
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_subdirectory( test_ros )
