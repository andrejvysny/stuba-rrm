

#include <visualization_msgs/Marker.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <ros/ros.h>
#include <thread>
#include "Robot.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "controlnode");
    ros::NodeHandle n;
    Robot robot(&n);
    std::thread publishThread(&Robot::publishMarker, &robot);
    std::thread moveRobotThread(&Robot::moveRobot, &robot);
    ros::AsyncSpinner spinner(2);
    spinner.start();


    ros::waitForShutdown(); // Block until shutdown signal is received
    return 0;
}
