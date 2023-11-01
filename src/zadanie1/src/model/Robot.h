#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H


class Robot {

public:
    static const int jointCount = 6;
    float getJointValue(int jointNumber);
    void setJointValue(int jointNumber, float value);

    void publishStates(ros::NodeHandle n);
private:
    float joints[Robot::jointCount] = {1.1,1.2,1.3,1.4,1.5,1.6}; // TODO remove init

    static bool validateJointNumber(int jointNumber);
};


#endif //SRC_ROBOT_H
