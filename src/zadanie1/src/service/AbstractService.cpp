//
// Created by andrejvysny on 1.11.2023.
//

#include "AbstractService.h"

#include <utility>
#include <ros/node_handle.h>


AbstractService::AbstractService(std::string serviceName, Robot *robotReference, ros::NodeHandle* n) {

    ROS_INFO("[SERVICE CREATED] %s",serviceName.c_str());


    this->serviceName = std::move(serviceName);
    this->robotReference = robotReference;


    ROS_INFO("[SERVICE] %s - Registering service");

    n->advertiseService(serviceName, callback);
    serviceServer = n->advertiseService("nazov1", &AbstractService::callback, this);

    serviceServer = n->advertiseService("move_absolute", callback);

    ros::spin();
}
