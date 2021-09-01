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

catkin_package(
  INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/submodules/optstop_cpp/include/
  include/
  LIBRARIES
  opstop
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

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} include)
add_subdirectory(${PROJECT_SOURCE_DIR}/submodules/pybind11)
add_subdirectory(${PROJECT_SOURCE_DIR}/submodules/optstop_cpp)

add_library(opstop_ros src/follow_joint_trajectory_action_wrapper.cpp)
target_link_libraries(opstop_ros ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} opstop)

add_executable(follow_joint_trajectory_wrapper
               src/follow_joint_trajectory_action_wrapper_node.cpp)

target_link_libraries(follow_joint_trajectory_wrapper ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES} opstop opstop_ros)

install(
  TARGETS opstop opstop_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS pyopstop
        LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/submodules/optstop_cpp/include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/opstop_ros/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(TARGETS follow_joint_trajectory_wrapper
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(
  PROGRAMS scripts/joint_trajectory_plotter scripts/tf_points_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
