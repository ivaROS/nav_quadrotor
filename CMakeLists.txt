cmake_minimum_required(VERSION 2.8.3)
project(nav_quadrotor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #Custom packages required
  benchmarking_tools
  fastmarch
  global_planner_plus
  pips_egocylindrical
  tf2_nav_msgs
  trajectory_based_nav
  trajectory_generator
  
  #
  angles
  trajectory_msgs
  rotors_control
  nodelet
  mav_msgs
  std_msgs
  costmap_2d
  dynamic_reconfigure
  message_generation
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   TrajectoryArray.msg
   CandidateWaypoint.msg
   CandidateWaypoints.msg
   WaypointProperty.msg
   DetailedWaypointArray.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   trajectory_msgs  # Or other packages containing msgs
   std_msgs
   geometry_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/NIConfigUtility.cfg
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
  INCLUDE_DIRS include
  LIBRARIES nav_quadrotor
  CATKIN_DEPENDS angles trajectory_generator trajectory_msgs rotors_control pips_egocylindrical message_runtime std_msgs global_planner_plus costmap_2d benchmarking_tools geometry_msgs tf2_geometry_msgs mav_msgs trajectory_based_nav
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  external/InscribedRectFinder
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/nav_quadrotor.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/nav_quadrotor_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


add_library(${PROJECT_NAME}
  src/basic_trajectory_wrapper.cpp
  src/basic_trajectory_source.cpp
  src/quadrotor_controller_interface.cpp
  src/extendable_lee_position_controller_node.cpp
  src/general_nav_impl.cpp
  src/global_impl.cpp
  src/global_goal_scoring.cpp
  src/multi_level_cc_wrapper.cpp
  src/new_trajectory_sources.cpp
  src/end_point_trajectory_source.cpp
  src/committed_global_goal_scoring.cpp
  src/detailed_waypoint_trajectory_source.cpp
  src/multi_level_trajectory_verifier.cpp
  src/inscribed_square_finder.cpp
  )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_global_impl_node
  src/global_impl_node.cpp
  )
add_dependencies(${PROJECT_NAME}_global_impl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME}_global_impl_node PROPERTIES OUTPUT_NAME global_impl_node PREFIX "")
target_link_libraries(${PROJECT_NAME}_global_impl_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_safety_controller_node
  src/safety_controller_node.cpp
  )
add_dependencies(${PROJECT_NAME}_safety_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME}_safety_controller_node PROPERTIES OUTPUT_NAME safety_controller_node PREFIX "")
target_link_libraries(${PROJECT_NAME}_safety_controller_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_library(${PROJECT_NAME}_fastmarch_nearest_depth
  src/fastmarch_nearest_depth.cpp
  )
add_dependencies(${PROJECT_NAME}_fastmarch_nearest_depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_fastmarch_nearest_depth
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_freespace_estimator
  src/freespace_estimator.cpp
  )
add_dependencies(${PROJECT_NAME}_freespace_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME}_freespace_estimator PROPERTIES OUTPUT_NAME freespace_estimator_node PREFIX "")
target_link_libraries(${PROJECT_NAME}_freespace_estimator
  ${PROJECT_NAME}_fastmarch_nearest_depth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

