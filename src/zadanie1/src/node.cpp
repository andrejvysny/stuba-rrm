#include <ros/ros.h>
#include <thread>
#include <sensor_msgs/JointState.h>
#include "model/Robot.h"
#include <rrm_msgs/Move.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "node");
    ros::NodeHandle n;

    Robot robot(&n);

    std::thread publishThread(&Robot::publishStates, &robot);

    ros::ServiceServer moveAbsoluteServer = n.advertiseService("/move_absolute", &Robot::moveAbsoluteCallback, &robot);
    ros::ServiceServer moveRelativeServer = n.advertiseService("/move_relative", &Robot::moveRelativeCallback, &robot);

    //ROS_INFO("Multiple Service Servers are ready.");
    // Use ros::AsyncSpinner to prevent blocking by multiple service servers
    ros::AsyncSpinner spinner(2); // 2 threads for 2 service servers
    spinner.start();

    ros::waitForShutdown(); // Block until shutdown signal is received
    return 0;
}
