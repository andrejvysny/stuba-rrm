#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cmath>
#include <fstream>

int main(int argc, char **argv)
{
    // Vytvorenie node a publishera
    ros::init(argc, argv, "trajectory_visualization");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);

    // Sprava pre trajektoriu
    moveit_msgs::RobotTrajectory trajectory;
    // Mena klbov musia byt vyplnene
    trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    // V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
    for (double t = 0; t <= 2; t += 0.1) {

        // Vytvorenie prejazdoveho bodu
        trajectory_msgs::JointTrajectoryPoint point;

        // Robot ma 6 klbov
        point.positions.resize(6);
        point.velocities.resize(6);
        point.accelerations.resize(6);

        // Klb 1
        point.positions[0] = t*0.5;
        point.velocities[0] = 0;
        point.accelerations[0] = 0;

        // Klb 2
        point.positions[1] = t*0.5;
        point.velocities[1] = 0;
        point.accelerations[1] =0;

        // Klb 3
        point.positions[2] = t*0.5;
        point.velocities[2] = 0;
        point.accelerations[2] = 0;

        // Klb 4
        point.positions[3] = t*0.5;
        point.velocities[3] = 0;
        point.accelerations[3] =0;

        // Klb 5
        point.positions[4] = t*0.5;
        point.velocities[4] = 0;
        point.accelerations[4] = 0;

        // Klb 6
        point.positions[5] = t*0.5;
        point.velocities[5] = 0;
        point.accelerations[5] =0;

        // Vlozenie casu prejazdu
        point.time_from_start = ros::Duration(t);

        // VLozenie bodu do trajektorie
        trajectory.joint_trajectory.points.push_back(point);
    }

    // Sprava pre vizualizaciu
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Potrebne vyplnit nazov modelu
    display_trajectory.model_id = "abb_irb";

    // Vlozenie vypocitanej trajektorie
    display_trajectory.trajectory.push_back(trajectory);

    // Publikuj trajektoriu kazde 2 sekundy
    ros::Rate loop_rate(0.5);
    while (ros::ok()) {
        publisher.publish(display_trajectory);
        loop_rate.sleep();
    }
    return 0;
}