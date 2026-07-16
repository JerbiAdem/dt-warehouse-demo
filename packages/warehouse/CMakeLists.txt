cmake_minimum_required(VERSION 3.0.2)
project(warehouse)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/warehouse_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
