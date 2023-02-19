cmake_minimum_required(VERSION 2.8.3)
project(myrobot_model)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  xacro
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Pose.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AttinyCommand.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myrobot_model
#  CATKIN_DEPENDS joint_state_publisher robot_state_publisher rviz xacro
#  DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)


find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(contact SHARED src/gazebo_plugins/ContactPlugin.cc)
target_link_libraries(contact ${GAZEBO_LIBRARIES})

add_library(gripper_move SHARED src/gazebo_plugins/gripper_move.cc)
target_link_libraries(gripper_move ${GAZEBO_LIBRARIES})

add_executable(attiny_server_sim src/attiny_server_sim.cpp)
target_link_libraries(attiny_server_sim ${catkin_LIBRARIES})
add_dependencies(attiny_server_sim ${catkin_EXPORTED_TARGETS})
