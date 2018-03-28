cmake_minimum_required(VERSION 2.8.3)
project(control_selector)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_selector
  CATKIN_DEPENDS nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(control_selector
             src/control_selector_node.cpp
             src/control.h
             src/voyagercontrol.h
             src/voyagercontrol.cpp
                                )

## Specify libraries to link a library or executable target against
target_link_libraries(control_selector
   ${catkin_LIBRARIES}
 )

