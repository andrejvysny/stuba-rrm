
#include <ros/ros.h>
#include <thread>
#include <sensor_msgs/JointState.h>
#include "model/Robot.h"
#include <rrm_msgs/Move.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "input");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<rrm_msgs::Move>("/move_relative");

    rrm_msgs::Move srv;
    srv.request.positions.resize(Robot::jointCount);

    for (int i=0; i<Robot::jointCount; i++){
        double number = 0.0;
        ROS_INFO("Enter how much should joint move  joint_%i: ", i+1);
        std::cin >> number;
        srv.request.positions.at(i) = number;
    }

    if (client.call(srv)) {
        ROS_INFO("result: %s", srv.response.message.c_str());
    } else {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    return 0;
}


