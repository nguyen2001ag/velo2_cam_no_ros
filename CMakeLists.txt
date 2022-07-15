cmake_minimum_required(VERSION 3.15)
project(velo2cam_calibration_no_ros)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -w ")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/")

## Find catkin macros and libraries
# find_package(cv_bridge REQUIRED)
# find_package(image_geometry REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(message_filters REQUIRED)
# find_package(tf REQUIRED)
# find_package(tf_conversions REQUIRED)
# find_package(cmake_modules REQUIRED)
# find_package(dynamic_reconfigure REQUIRED)
# find_package(TinyXML REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED common io)
find_package(Eigen3 REQUIRED)



find_package(ROS REQUIRED COMPONENTS 
roscpp 
tf
cv_bridge 
pcl_ros 
message_filters 
pcl_conversions)

###########
## Build ##
###########
# include_directories(
#   include
#   ${OpenCV_LIBRARIES}
#   ${TinyXML_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS} 
# )

add_library(mono_qr_pattern src/mono_qr_pattern.cpp)

target_link_libraries(mono_qr_pattern 
PUBLIC
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ros::pcl_conversions
  ros::pcl_ros
)

target_include_directories(mono_qr_pattern
PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(lidar_pattern src/lidar_pattern.cpp)

target_link_libraries(lidar_pattern 
PUBLIC
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ros::pcl_conversions
  ros::pcl_ros
)

target_include_directories(lidar_pattern
PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(stereo_pattern src/lidar_pattern.cpp)

target_link_libraries(stereo_pattern 
PUBLIC
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ros::pcl_conversions
  ros::pcl_ros
)

target_include_directories(stereo_pattern
PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(calibration_server src/calibration_server.cpp)

target_link_libraries(calibration_server 
PUBLIC
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${TinyXML_LIBRARIES}
  ${Eigen3_LIBRARIES}
  stereo_pattern
  lidar_pattern
  mono_qr_pattern
)

target_include_directories(calibration_server
PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${TinyXML_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

#link_directories()

#add_definitions()

## Declare cpp executables
# add_executable(lidar_pattern src/lidar_pattern.cpp)
# add_executable(stereo_pattern src/stereo_pattern.cpp)
add_executable(tester src/tester.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
target_include_directories(tester
PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${TinyXML_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


target_link_libraries(tester
mono_qr_pattern
lidar_pattern
stereo_pattern
calibration_server
ros::roscpp 
ros::tf
ros::cv_bridge 
ros::pcl_ros
ros::message_filters
ros::pcl_conversions
)

