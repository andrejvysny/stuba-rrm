#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H
#include <visualization_msgs/Marker.h>
#include <string>

class Robot {

public:
    explicit Robot(ros::NodeHandle* n);

    void moveRobot();
    void publishMarker();

    geometry_msgs::Pose generatePosition(double xPosition,double yPosition,double zPosition,double xOrientation,double yOrientation,double zOrientation,double wOrientation);

private:

    visualization_msgs::Marker getMarker();
    ros::NodeHandle* nodeHandle;

};


#endif //SRC_ROBOT_H