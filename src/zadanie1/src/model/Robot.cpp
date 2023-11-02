#include <exception>
#include <ros/ros.h>
#include <string>
#include "Robot.h"
#include <sensor_msgs/JointState.h>
#include <rrm_msgs/Move.h>


void Robot::setJointValue(int jointNumber, double value) {
    if(!Robot::validateJointNumber(jointNumber)){
        throw std::exception();
    }
    this->joints[jointNumber-1] = value;
}

double Robot::getJointValue(int jointNumber) {
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

void Robot::publishStates() {
    ros::Publisher publisher = this->nodeHandle->advertise<sensor_msgs::JointState>("/joint_states", 10);
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

Robot::Robot(ros::NodeHandle* n) {

    this->nodeHandle = n;

    for (int i = 0; i < Robot::jointCount; i++){
        this->nodeHandle->getParam("joint" + std::to_string(i+1) + "/lower_limit", this->jointLimits[i][0]);
        this->nodeHandle->getParam("joint" + std::to_string(i+1) + "/upper_limit", this->jointLimits[i][1]);
    }

}


bool Robot::moveAbsoluteCallback(rrm_msgs::Move::Request &req, rrm_msgs::Move::Response &res) {
    if (req.positions.size() != Robot::jointCount){
        res.success = false;
        res.message = "Invalid input!";
        return true;
    }
    //ROS_INFO("[SERVICE] /move_absolute called I receive positions [%f,%f,%f,%f,%f,%f]", req.positions[0], req.positions[1], req.positions[2], req.positions[3], req.positions[4], req.positions[5]);

    for (int i = 0; i < Robot::jointCount; i++){

        if (!this->insideLimit(req.positions[i],i)){
            res.success = false;
            res.message = "Out of range";
            return true;
        }
        this->setJointValue(i+1, req.positions[i]);
    }
    res.success = true;
    res.message = "Joints changed!";
    //ROS_INFO("[SERVICE] Robot positions [%f,%f,%f,%f,%f,%f]", robot.getJointValue(1),robot.getJointValue(2),robot.getJointValue(3),robot.getJointValue(4),robot.getJointValue(5),robot.getJointValue(6));

    return true;
}

bool Robot::moveRelativeCallback(rrm_msgs::Move::Request &req, rrm_msgs::Move::Response &res) {
    if (req.positions.size() != Robot::jointCount){
        res.success = false;
        res.message = "Invalid input!";
        return true;
    }


    //fistly check
    double newValue;


    for (int i = 0; i < Robot::jointCount; i++){

        newValue = this->getJointValue(i+1) + req.positions[i];

        if (!this->insideLimit(newValue,i)){
            res.success = false;
            res.message = "Out of range";
            return true;
        }

        //ROS_INFO("Joint_%d prevValue: %5f, request: %5f, newValue: %5f, limits: <%5f, %5f>, InsideLimit: %s",i+1,prev,req.positions[i],newValue,this->jointLimits[i][0],this->jointLimits[i][1],(this->insideLimit(newValue,i) ? "INSIDE":"OUTSIDE"));
    }

    for (int i = 0; i < Robot::jointCount; i++){
        double prev = getJointValue(i+1);
        newValue = this->getJointValue(i+1) + req.positions[i];
        ROS_INFO("Joint_%d prevValue: %5f, request: %5f, newValue: %5f, limits: <%5f, %5f>, InsideLimit: %s",i+1,prev,req.positions[i],newValue,this->jointLimits[i][0],this->jointLimits[i][1],(this->insideLimit(newValue,i) ? "INSIDE":"OUTSIDE"));
        this->setJointValue(i+1, newValue);
    }
    res.success = true;
    res.message = "Joints changed!";
    ROS_INFO("--------------------------");

    //ROS_INFO("[SERVICE] /move_relative request [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", req.positions[0], req.positions[1], req.positions[2], req.positions[3], req.positions[4], req.positions[5]);
    //ROS_INFO("[SERVICE] /move_relative Robot   [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", getJointValue(1),getJointValue(2),getJointValue(3),getJointValue(4),getJointValue(5),getJointValue(6));

    return true;
}

bool Robot::insideLimit(double valueToCheck, int jointNumber) {

    bool lowerLimit = (valueToCheck + FLT_EPSILON) >= this->jointLimits[jointNumber][0] ;
    bool upperLimit = (valueToCheck - FLT_EPSILON) <= this->jointLimits[jointNumber][1] ;

    if(!lowerLimit){
        ROS_INFO("Joint_%d LowerLimit: %f, DATA: %f",jointNumber+1,this->jointLimits[jointNumber][0],valueToCheck);
        return false;
    }

    if(!upperLimit){
        ROS_INFO("Joint_%d UpperLimit: %f, DATA: %f",jointNumber+1,this->jointLimits[jointNumber][1],valueToCheck);
        return false;
    }
    return true;
}


