#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "model/Robot.h"
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);


int main(int argc, char **argv) {
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_states", 10, jointStateCallback);
    ros::spin();
    return 0;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    for (int i=0; i<Robot::jointCount; i++){
        ROS_INFO("JOINT: %s = %f", msg->name.at(i).c_str(),msg->position.at(i));
    }

    ROS_INFO("----------------");

}