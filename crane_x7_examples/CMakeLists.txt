cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)
add_service_files(FILES ObstacleAvoidance.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

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
#  LIBRARIES crane_x7_examples
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
 DEPENDS
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLDUE_DIRS})


#############
## Install ##
#############

add_executable(cam_ex   scripts/camera.cpp)
target_link_libraries(cam_ex ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
##add_executable(cam   scripts/camera_sub.cpp)
##target_link_libraries(cam2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
install(PROGRAMS
  scripts/crane_x7_pick_and_place_demo.py
  scripts/realsense_camera.py
  scripts/gripper_action_example.py
  scripts/pose_groupstate_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)