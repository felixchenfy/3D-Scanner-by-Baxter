cmake_minimum_required(VERSION 2.8.3)
project(scan3d_by_baxter)

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
# set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin ) 
# set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

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



############### ROS ######################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  
  std_msgs
  sensor_msgs
  geometry_msgs

  message_generation

  pcl_ros

)
add_message_files( # add my message
  FILES
  T4x4.msg
)
add_service_files(
  FILES
  PrintBaxterGripperPose.srv
  PrintBaxterJointAngles.srv
)
generate_messages( # dependencies to my message
  DEPENDENCIES
  std_msgs
  geometry_msgs
  baxter_core_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime) # declares dependencies for packages that depend on this package.

include_directories(
  ${catkin_INCLUDE_DIRS}
)

############### Sub directories ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src_cpp )
add_subdirectory( test )
add_subdirectory( test_ros )
add_subdirectory( src_main )
