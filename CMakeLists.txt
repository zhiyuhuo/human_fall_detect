cmake_minimum_required(VERSION 2.8.3)
project(human_fall_detect)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  std_msgs
  image_transport
  dynamic_reconfigure
  sensor_msgs
  tf
  visualization_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  visualization_msgs
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(PCL 1.7 REQUIRED )

##Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp rospy dynamic_reconfigure std_msgs image_transport dynamic_reconfigure sensor_msgs tf visualization_msgs cv_bridge pcl_conversions pcl_ros visualization_msgs)


include_directories(include
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

link_directories(
${PCL_LIBRARY_DIRS} 
${OpenCV_LIBS} )

add_definitions(${PCL_DEFINITIONS})

###########
## Build ##
###########

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/HumanDetector.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/FallDetector.cpp
        )

###########my output
add_executable(human_fall_detect_node src/main.cc)
target_link_libraries(human_fall_detect_node ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

###########my output end



















