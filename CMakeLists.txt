cmake_minimum_required(VERSION 2.8.3)
project(ground_station)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)
catkin_package()
catkin_python_setup()

#
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/ground_station
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
