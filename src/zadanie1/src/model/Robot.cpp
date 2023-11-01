#include <exception>
#include <ros/ros.h>
#include <string>
#include "Robot.h"
#include <sensor_msgs/JointState.h>


void Robot::setJointValue(int jointNumber, float value) {
    if(!Robot::validateJointNumber(jointNumber)){
        throw std::exception();
    }
    this->joints[jointNumber-1] = value;
}

float Robot::getJointValue(int jointNumber) {
    if(!Robot::validateJointNumber(jointNumber)){
        throw std::exception();
    }
    return this->joints[jointNumber-1];
}

bool Robot::validateJointNumber(int jointNumber) {
    if(jointNumber-1 > Robot::jointCount || jointNumber < 1){ //TODO: check this condition - not sure is right
        return false;
    }
    return true;
}

void Robot::publishStates(ros::NodeHandle n) {
    ros::Publisher publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name.resize(Robot::jointCount);
        msg.position.resize(Robot::jointCount);
        for (int i=0; i<Robot::jointCount; i++){
            msg.name.at(i) = "joint_" + std::to_string(i+1);
            msg.position.at(i) = this->getJointValue(i+1);
            //  ROS_INFO("PUBLISH JOINT: %s = %f", msg.name.at(i).c_str(), msg.position.at(i));
        }
        //ROS_INFO("___________");
        publisher.publish(msg);
        loop_rate.sleep();
    }
}
