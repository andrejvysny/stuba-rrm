cmake_minimum_required(VERSION 3.0.2)
project(acceptance_test_rrm_1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rostest
)

catkin_package()

catkin_install_python(PROGRAMS
        test/test_rrm1.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_rostest(test/rrm1.test)