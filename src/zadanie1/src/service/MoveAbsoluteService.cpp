//
// Created by andrejvysny on 1.11.2023.
//

#include "MoveAbsoluteService.h"

MoveAbsoluteService::MoveAbsoluteService(Robot *robotReference,ros::NodeHandle* n) : AbstractService("/move_absolute",robotReference,n) {


}

bool MoveAbsoluteService::callback() {


    return true;
}