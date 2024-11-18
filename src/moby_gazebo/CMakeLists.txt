cmake_minimum_required(VERSION 2.8.3)
project(moby_gazebo)

# find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy nav_msgs geometry_msgs std_msgs tf)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  src/swerve_drive_controller.py
  src/teleop_gazebo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
