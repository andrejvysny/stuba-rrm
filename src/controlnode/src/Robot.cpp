#include <exception>
#include <ros/ros.h>
#include <string>
#include "Robot.h"

#include <visualization_msgs/Marker.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


visualization_msgs::Marker Robot::getMarker(){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "meshes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 1.0;
    marker.pose.position.y = 0.5;
    marker.pose.position.z = 0.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.4;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.mesh_resource = "file:///home/andrejvysny/catkin_ws/src/controlnode/truck.stl";

    return marker;
}

void Robot::publishMarker() {
    ros::Publisher publisher = this->nodeHandle->advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    visualization_msgs::Marker marker = this->getMarker();

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        publisher.publish(marker);
        loop_rate.sleep();
    }
}


geometry_msgs::Pose Robot::generatePosition(double xPosition,double yPosition,double zPosition,double xOrientation,double yOrientation,double zOrientation,double wOrientation){
    geometry_msgs::Pose position;
    position.orientation.x = xOrientation;
    position.orientation.y = yOrientation;
    position.orientation.z = zOrientation;
    position.orientation.w = wOrientation;
    position.position.x = xPosition;
    position.position.y = yPosition;
    position.position.z = zPosition;

    return position;

}

/*
 * Phase One
 *
 *
 *
 */


void Robot::moveRobot(){

    geometry_msgs::Pose homePosition = this->generatePosition(1.2699, 2.0596e-06, 1.5701,-5.6371e-06, 0.70711, -4.491e-06, 0.70711);


    static const std::string PLANNING_GROUP = "default";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan plan;



    move_group_interface.setPoseTarget(
            this->generatePosition(1.3913, 0.40143, 1.0991,-0.0039903, 0.95261, 0.010869, 0.30397)
    );

    if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS){
        move_group_interface.move();
    }else{
        ROS_INFO("Error planning");
    }



    move_group_interface.setPoseTarget(
            this->generatePosition(1.453, -0.075356, 1.1288, -0.0044783, 0.95249, 0.011199, 0.30433)
    );

    if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS){
        move_group_interface.move();
    }else{
        ROS_INFO("Error planning");
    }


    move_group_interface.setPoseTarget(
            homePosition
    );

    if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS){
        move_group_interface.move();
    }else{
        ROS_INFO("Error planning");
    }




    move_group_interface.setPoseTarget(
            this->generatePosition(1.0057, -0.49844, 0.83726,8.0762e-05, 0.90674, -0.00021898, 0.42169)
    );

    if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS){
        move_group_interface.move();
    }else{
        ROS_INFO("Error planning");
    }



    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(this->generatePosition(1.1362, -0.59457, 0.96394,-0.00048496, 0.90695, 3.1319e-05, 0.42125));
    waypoints.push_back(this->generatePosition(1.1343, -0.84865, 0.96161,-0.0004795, 0.9069, -3.2872e-05, 0.42135));


    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group_interface.execute(trajectory);
// WORKING






}

Robot::Robot(ros::NodeHandle* n) {

    this->nodeHandle = n;

}


