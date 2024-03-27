cmake_minimum_required(VERSION 3.0.2)
project(opstop_ros)
add_compile_options(-std=c++17)

find_package(
  catkin REQUIRED
  COMPONENTS actionlib
             actionlib_msgs
             control_msgs
             gsplines_follow_trajectory
             gsplines_msgs
             gsplines_ros
             joint_trajectory_controller
             roscpp
             rospy
             std_msgs
             trajectory_msgs)

find_package(Eigen3 REQUIRED)
find_package(GSplines REQUIRED)
find_package(OpStop REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include/
  LIBRARIES
  opstop_ros
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  control_msgs
  gsplines_follow_trajectory
  gsplines_msgs
  gsplines_ros
  joint_trajectory_controller
  roscpp
  rospy
  std_msgs
  trajectory_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS}
  ${gsplines_INCLUDE_DIRS} ${opstop_INCLUDE_DIRS} include)

add_library(opstop_ros src/follow_joint_trajectory_action_wrapper.cpp)
target_link_libraries(
  opstop_ros ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${gsplines_LIBRARIES}
  ${opstop_LIBRARIES} ${pinocchio_LIBRARIES})

add_executable(follow_joint_trajectory_wrapper
               src/follow_joint_trajectory_action_wrapper_node.cpp)

target_link_libraries(
  follow_joint_trajectory_wrapper
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${gsplines_LIBRARIES}
  ${opstop_LIBRARIES}
  ${pinocchio_LIBRARIES}
  opstop_ros)

install(
  TARGETS opstop_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/opstop_ros/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(TARGETS follow_joint_trajectory_wrapper
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(
  PROGRAMS scripts/joint_trajectory_plotter scripts/timed_stopper DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})
