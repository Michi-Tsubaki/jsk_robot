if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(jsk_baxter_startup)
  find_package(ament_cmake REQUIRED)
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
  install(PROGRAMS scripts/bridge.sh DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS scripts/ros1_state_adapter.py DESTINATION share/${PROJECT_NAME}/ros1)
  ament_package()
else() # if("$ENV{ROS_VERSION}" EQUAL 2)
cmake_minimum_required(VERSION 2.8.3)
project(jsk_baxter_startup)

find_package(catkin)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_baxter_startup
#  CATKIN_DEPENDS baxter_interface baxter_tools openni_launch
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_add_env_hooks(99.jsk_baxter_startup SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY config jsk_baxter_joy jsk_baxter_lifelog jsk_baxter_moveit jsk_baxter_sensors jsk_baxter_tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES baxter.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES launch/baxter_ros1.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
catkin_install_python(PROGRAMS scripts/ros1_state_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
