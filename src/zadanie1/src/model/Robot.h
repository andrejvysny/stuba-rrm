#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H
#include <rrm_msgs/Move.h>


class Robot {

public:

    Robot(ros::NodeHandle* n);

    static const int jointCount = 6;
    float getJointValue(int jointNumber);
    void setJointValue(int jointNumber, float value);

    bool moveRelativeCallback(rrm_msgs::Move::Request &req, rrm_msgs::Move::Response &res);
    bool moveAbsoluteCallback(rrm_msgs::Move::Request &req, rrm_msgs::Move::Response &res);

    void publishStates();
private:

    ros::NodeHandle* nodeHandle;
    float joints[Robot::jointCount] = {0,0,0,0,0,0}; // TODO remove init
    float jointLimits[Robot::jointCount][2];


    bool insideLimit(float valueToCheck, int jointNumber);
    static bool validateJointNumber(int jointNumber);
};


#endif //SRC_ROBOT_H