//
// Created by andrejvysny on 1.11.2023.
//

#ifndef CATKIN_WS_ABSTRACTSERVICE_H
#define CATKIN_WS_ABSTRACTSERVICE_H


#include <string>
#include <ros/node_handle.h>
#include "../model/Robot.h"

class AbstractService {
public:
    AbstractService(std::string serviceName, Robot* robotReference, ros::NodeHandle* n);
    virtual bool callback() = 0;

    void registerCallback(auto callbackClass, auto object);
protected:
    Robot* robotReference;
    std::string serviceName;
    ros::ServiceServer serviceServer;

};



#endif //CATKIN_WS_ABSTRACTSERVICE_H
