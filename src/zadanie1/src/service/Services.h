//
// Created by andrejvysny on 2.11.2023.
//

#ifndef CATKIN_WS_SERVICES_H
#define CATKIN_WS_SERVICES_H

#include <ros/ros.h>
#include "../model/Robot.h"
#include <rrm_msgs/Move.h>

class Services {
public:
    explicit Services(ros::NodeHandle* n, Robot* robot);
private:
    bool moveAbsoluteCallback(rrm_msgs::Move::Request& req, rrm_msgs::Move::Response& res);
    bool moveRelativeCallback(rrm_msgs::Move::Request& req, rrm_msgs::Move::Response& res);
    Robot* robot;
};


#endif //CATKIN_WS_SERVICES_H
