cmake_minimum_required(VERSION 3.0.2)
project(ik_solver_example)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        eigen_conversions
        moveit_ros_planning_interface
        )

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(ik_solver_example src/ik_solver_example.cpp)
add_dependencies(ik_solver_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ik_solver_example
        ${catkin_LIBRARIES}
        )