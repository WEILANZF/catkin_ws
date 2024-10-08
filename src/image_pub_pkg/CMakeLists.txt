cmake_minimum_required(VERSION 3.0.2)
project(image_pub_pkg)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
)

find_package(OpenCV)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_pub_pkg
#  CATKIN_DEPENDS cv_bridge image_transport opencv2 rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(image_publisher src/image_publish.cpp)
target_link_libraries(image_publisher ${catkin_LIBRARIES})
add_executable(image_publisher2 src/image_publish2.cpp)
target_link_libraries(image_publisher2 ${catkin_LIBRARIES})
add_executable(image_publisher3 src/image_publish3.cpp)
target_link_libraries(image_publisher3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(image_publisher4 src/image_publish4.cpp)
target_link_libraries(image_publisher4 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(image_publisher5 src/image_publish5.cpp)
target_link_libraries(image_publisher5 ${catkin_LIBRARIES} ${OpenCV_LIBS})
