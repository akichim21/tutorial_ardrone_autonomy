cmake_minimum_required(VERSION 2.8.3)
project(tutorial_ardrone_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
)
find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tutorial_ardrone_autonomy
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(takeoff_and_land src/takeoff_and_land.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(takeoff_and_land ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(takeoff_and_land ${catkin_LIBRARIES})

add_executable(opencv src/opencv.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(opencv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(opencv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

