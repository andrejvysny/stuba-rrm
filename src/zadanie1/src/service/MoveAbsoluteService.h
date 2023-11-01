//
// Created by andrejvysny on 1.11.2023.
//

#ifndef CATKIN_WS_MOVEABSOLUTESERVICE_H
#define CATKIN_WS_MOVEABSOLUTESERVICE_H


#include "AbstractService.h"

class MoveAbsoluteService : public AbstractService{

public:
    explicit MoveAbsoluteService(Robot *robotReference, ros::NodeHandle* n);

    bool callback() override;

};


#endif //CATKIN_WS_MOVEABSOLUTESERVICE_H
