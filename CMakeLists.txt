cmake_minimum_required(VERSION 2.8.3)
project(synchronize_pointclouds)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    cv_bridge
    image_transport
)

## System dependencies 
find_package(OpenCV 3.3.1 EXACT REQUIRED)
find_package(CUDA 10.1 EXACT REQUIRED)
find_package(PCL 1.9.1 EXACT REQUIRED)

## The catkin_package macro generates cmake config files for your package
catkin_package()

## Specify additional locations of header files
set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )
set(PCL_CUDA_INCLUDE_DIRS /home/csrobot/pcl/pcl-pcl-1.9.1/cuda/io)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PCL_CUDA_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

MESSAGE("PCL_CUDA_INCLUDE_DIRS:= ${PCL_CUDA_INCLUDE_DIRS}")

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message("pcl_ros:=${catkin_INCLUDE_DIRS}")
message("pcl_library_shit:=${PCL_LIBRARY_DIRS}")
message("pcl_definitions:=${PCL_DEFINITIONS}")

## Declare a C++ executable
cuda_add_executable(cloud_node  src/MLS.h
                                src/cloud_node.cpp 
                                src/cloud_class.cpp 
                                src/cloud_class.hpp 
                                src/MLS.cu)
add_executable(transformer_node src/transformer_node.cpp)
add_executable(ufo_transformer_node src/ufo_transformer_node.cpp)
add_executable(receiver_node src/receiver_node.cpp)
#add_executable(calibration_node src/calibration_node.cpp)

## Specify libraries to link a library or executable target against
#message("pcl:=${PCL_LIBRARIES}")
message("pcl:=${PCL_INCLUDE_DIRS}")
target_link_libraries(cloud_node    ${catkin_LIBRARIES} 
                                    ${PCL_LIBRARIES} 
                                    ${PCL_CUDA_COMMON_LIBRARIES})
target_link_libraries(transformer_node ${catkin_LIBRARIES})
target_link_libraries(ufo_transformer_node ${catkin_LIBRARIES})
target_link_libraries(receiver_node ${catkin_LIBRARIES})
#target_link_libraries(calibration_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

