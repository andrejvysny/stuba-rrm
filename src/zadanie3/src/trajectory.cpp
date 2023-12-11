#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cmath>
#include <fstream>

double getPosition(double t, double a[6]) {
    return a[0] + a[1] * t + a[2] * pow(t, 2) + a[3] * pow(t, 3) + +a[4] * pow(t, 4) + a[5] * pow(t, 5);
}

double getVelocity(double t, double a[6]) {
    return 0 + a[1] + 2 * a[2] * t + 3 * a[3] * pow(t, 2) + +4 * a[4] * pow(t, 3) + 5 * a[5] * pow(t, 4);
}

double getAcceleration(double t, double a[6]) {
    return 0 + 0 + 2 * a[2] + 6 * a[3] * t + 12 * a[4] * pow(t, 2) + 20 * a[5] * pow(t, 3);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_visualization");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};


    double a1[6] = {
            0, 0, 0, 0.24543693, -0.09203885, 0.00920388
    };
    double a3_1[6] = {
            0, 0, 0, 5.23598776, -7.85398163, 3.14159265
    };

    double a3_2[6] = {
            0.82741535, -1.03426919, 1.29283648, -0.71106007, 0.16160456, -0.01292836
    };

    for (double t = 0; t <= 4; t += 0.1) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(6);
        point.velocities.resize(6);
        point.accelerations.resize(6);

        // Klb 1
        point.positions[0] = getPosition(t, a1);
        point.velocities[0] = getVelocity(t, a1);
        point.accelerations[0] = getAcceleration(t, a1);


        // Klb 2
        point.positions[1] = 0;
        point.velocities[1] = 0;
        point.accelerations[1] = 0;

        // Klb 3
        point.positions[2] = getPosition(t, t <= 1 ? a3_1 : a3_2);
        point.velocities[2] = getVelocity(t, t <= 1 ? a3_1 : a3_2);
        point.accelerations[2] = getAcceleration(t, t <= 1 ? a3_1 : a3_2);

        // Klb 4
        point.positions[3] = 0;
        point.velocities[3] = 0;
        point.accelerations[3] = 0;

        // Klb 5
        point.positions[4] = 0;
        point.velocities[4] = 0;
        point.accelerations[4] = 0;

        // Klb 6
        point.positions[5] = 0;
        point.velocities[5] = 0;
        point.accelerations[5] = 0;

        point.time_from_start = ros::Duration(t);
        trajectory.joint_trajectory.points.push_back(point);
    }

    // Sprava pre vizualizaciu
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Potrebne vyplnit nazov modelu
    display_trajectory.model_id = "abb_irb";

    // Vlozenie vypocitanej trajektorie
    display_trajectory.trajectory.push_back(trajectory);


    ros::Rate loop_rate(0.5);
    while (ros::ok()) {
        publisher.publish(display_trajectory);
        loop_rate.sleep();
    }
    return 0;
}