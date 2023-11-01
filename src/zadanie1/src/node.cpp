#include <ros/ros.h>
#include <thread>
#include <sensor_msgs/JointState.h>
#include "model/Robot.h"
#include <rrm_msgs/Move.h>


Robot robot;


bool moveAbsoluteCallback(rrm_msgs::Move::Request& req, rrm_msgs::Move::Response& res) {
    int length = sizeof(req.positions) / sizeof(int);
    if (length<Robot::jointCount){
        res.success = false;
        res.message = "Invalid input!";
        return true;
    }
    ROS_INFO("[SERVICE] /move_absolute called I receive positions [%f,%f,%f,%f,%f,%f]", req.positions[0], req.positions[1], req.positions[2], req.positions[3], req.positions[4], req.positions[5]);

    for (int i = 0; i < Robot::jointCount; i++){
        robot.setJointValue(i+1, req.positions[i]);
    }
    res.success = true;
    res.message = "Joints changed!";
    ROS_INFO("[SERVICE] Robot positions [%f,%f,%f,%f,%f,%f]", robot.getJointValue(1),robot.getJointValue(2),robot.getJointValue(3),robot.getJointValue(4),robot.getJointValue(5),robot.getJointValue(6));

    return true;
}


bool moveRelativeCallback(rrm_msgs::Move::Request& req, rrm_msgs::Move::Response& res) {
    int length = sizeof(req.positions) / sizeof(int);
    if (length<Robot::jointCount){
        res.success = false;
        res.message = "Invalid input!";
        return true;
    }
    ROS_INFO("[SERVICE] /move_relative called I receive positions [%f,%f,%f,%f,%f,%f]", req.positions[0], req.positions[1], req.positions[2], req.positions[3], req.positions[4], req.positions[5]);

    for (int i = 0; i < Robot::jointCount; i++){
        robot.setJointValue(i+1, robot.getJointValue(i+1) + req.positions[i]);
    }
    res.success = true;
    res.message = "Joints changed!";
    ROS_INFO("[SERVICE] Robot positions [%f,%f,%f,%f,%f,%f]", robot.getJointValue(1),robot.getJointValue(2),robot.getJointValue(3),robot.getJointValue(4),robot.getJointValue(5),robot.getJointValue(6));

    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "node");
    ros::NodeHandle n;

    std::thread publishThread(&Robot::publishStates, &robot, n);

    ros::ServiceServer moveAbsoluteServer = n.advertiseService("/move_absolute", moveAbsoluteCallback);
    ros::ServiceServer moveRelativeServer = n.advertiseService("/move_relative", moveRelativeCallback);
    ROS_INFO("Multiple Service Servers are ready.");
    // Use ros::AsyncSpinner to prevent blocking by multiple service servers
    ros::AsyncSpinner spinner(2); // 2 threads for 2 service servers
    spinner.start();

    ros::waitForShutdown(); // Block until shutdown signal is received
    return 0;
}
