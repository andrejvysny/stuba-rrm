#include <ros/ros.h>
#include <thread>
#include <sensor_msgs/JointState.h>
#include "model/Robot.h"
#include "service/Services.h"
#include <rrm_msgs/Move.h>







int main(int argc, char **argv) {
    ros::init(argc, argv, "node");
    ros::NodeHandle n;
    Robot robot;

    Services services(&n,&robot);

    std::thread publishThread(&Robot::publishStates, &robot, n);



    ros::waitForShutdown(); // Block until shutdown signal is received
    return 0;
}
