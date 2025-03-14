cmake_minimum_required(VERSION 3.0.2)
project(ply-measure-demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()
catkin_package()

set(SRC_FILES
  scripts/react.py
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ${SRC_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )