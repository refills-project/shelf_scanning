cmake_minimum_required(VERSION 2.8.3)
project(barcode_detector)

## Make sure HALCONROOT is set
if(NOT DEFINED ENV{HALCONROOT})
  message( FATAL_ERROR "PLEASE SET HALCONROOT" )
endif()



add_definitions(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  asr_halcon_bridge 
  cv_bridge
  image_transport
  sensor_msgs
  refills_msgs
)

find_package( OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES barcode_detector2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  $ENV{HALCONROOT}/include
  $ENV{HALCONROOT}/include/halconcpp
  ${catkin_INCLUDE_DIRS}
  )

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

  #From Benny for include_directories
  #${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

link_directories($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})




add_executable(barcode_detector src/barcode_hack)


target_link_libraries(barcode_detector
   halconcpp
   halcon
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_dependencies(barcode_detector ${catkin_EXPORTED_TARGETS})
