#ifndef CATKIN_WS_ROBOT_H
#define CATKIN_WS_ROBOT_H

#include <ros/ros.h>


class Robot {


public:
    explicit Robot(ros::NodeHandle* n);

private:
    ros::NodeHandle* nodeHandle;


    int getTrajectory();
    void calculateTrajectory();

};


#endif //CATKIN_WS_ROBOT_H