cmake_minimum_required(VERSION 2.8.3)
project(twist_controller)

## Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++11)
add_compile_options(-std=c++11 -O2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dbw_mkz_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  styx_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Third party dependencies
find_package(cmake_modules REQUIRED)

find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})

# To pull sources using Git
include(ExternalProject) 


set(CPPAD_SRC_LOCATION ${CMAKE_BINARY_DIR}/../devel/coin-or-libs/cppad)
set(IPOPT_SRC_LOCATION ${CMAKE_BINARY_DIR}/../devel/coin-or-libs/ipopt)


# Get source files for Model predictive control optimization
# Import Coin-Or library for Interior Point Optimization
ExternalProject_Add(Get_Coin_Or_Ipopt
    GIT_REPOSITORY https://github.com/coin-or/Ipopt.git
    DOWNLOAD_DIR ${IPOPT_SRC_LOCATION}/src
    SOURCE_DIR ${IPOPT_SRC_LOCATION}/src
    BINARY_DIR ${IPOPT_SRC_LOCATION}/build
    CONFIGURE_COMMAND ../src/configure
    BUILD_COMMAND make
    INSTALL_COMMAND make install    
)

# Get source files for Model predictive control optimization
# Import Coin-Or library for Automatic differentiation
ExternalProject_Add(Get_Coin_Or_CppAD
    DEPENDS Get_Coin_Or_Ipopt
    GIT_REPOSITORY https://github.com/coin-or/CppAD.git
    DOWNLOAD_DIR ${CPPAD_SRC_LOCATION}/src
    SOURCE_DIR ${CPPAD_SRC_LOCATION}/src
    BINARY_DIR ${CPPAD_SRC_LOCATION}/build
    CMAKE_GENERATOR "Unix Makefiles"
    CMAKE_ARGS -D CMAKE_BUILD_TYPE=RELEASE
    CMAKE_ARGS -D cppad_prefix=${CPPAD_SRC_LOCATION}
    CMAKE_ARGS -D ipopt_prefix=${IPOPT_SRC_LOCATION}/build
)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   dbw_mkz_msgs#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES twist_controller
#  CATKIN_DEPENDS dbw_mkz_msgs geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)





###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


# Include libraries for Model predictive control optimization
include_directories(${CPPAD_SRC_LOCATION}/include ${IPOPT_SRC_LOCATION}/build/include)
link_directories(${CPPAD_SRC_LOCATION}/lib ${IPOPT_SRC_LOCATION}/build/lib)


# Define set of files
set(SRCS
  src/LowLevelControllerNode.cpp
)

## Declare a C++ library
add_library(low_level_controller ${SRCS})

target_link_libraries(low_level_controller
  ${catkin_LIBRARIES}
  libcppad_ipopt.a
  )

add_custom_target(Get_Coin_Or_Math_Libs ALL DEPENDS Get_Coin_Or_CppAD)

add_executable(low_level_controller_node src/main.cpp)
target_link_libraries(low_level_controller_node low_level_controller)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Forcing to retrieve maths coin-or libraries before compiling the node
add_dependencies(low_level_controller Get_Coin_Or_Math_Libs)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/twist_controller_node.cpp)

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

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_twist_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
