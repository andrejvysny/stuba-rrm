#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cmath>
#include <fstream>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <math.h>




std::vector<std::vector<double>> calculateSolutionsForTargetPosition(Eigen::Affine3d target);


double getPosition(double t, Eigen::VectorXd a) {
    return a[0] + a[1] * t + a[2] * pow(t, 2) + a[3] * pow(t, 3) + +a[4] * pow(t, 4) + a[5] * pow(t, 5);
}

Eigen::MatrixXd getM6(double t0, double tf) {
    Eigen::MatrixXd m(6, 6);
    m << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5),
            0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4),
            0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3),
            1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5),
            0, 1, 2 * tf, 3 * pow(tf, 2), 4 * pow(tf, 3), 5 * pow(tf, 4),
            0, 0, 2, 6 * tf, 12 * pow(tf, 2), 20 * pow(tf, 3);

    return m;
}

Eigen::VectorXd getV(double m, double md, double mdd, double n, double nd, double ndd) {
    Eigen::VectorXd v(6);
    v << m, md, mdd, n, nd, ndd;
    v.transpose();
    return v;
}

Eigen::VectorXd  calcAParameters(double t0, double tf, double param0, double paramf) {
    Eigen::VectorXd a;
    Eigen::VectorXd v = getV(param0, 0,0,paramf,0,0);
    Eigen::MatrixXd m = getM6(t0,tf);
    m = m.inverse();
    a = m*v;
    return a;
}


Eigen::VectorXd getYParameterForT(double t){

    double t0,tf,param0,paramf;

    if (t < 1){
        t0 = 0;
        tf = 1;
        param0 = 0;
        paramf = 0;

    }else if (t >= 1 && t < 2){
        t0 = 1;
        tf = 2;
        param0 = 0;
        paramf = 0;
    }else if (t >= 2 && t < 3){
        t0 = 2;
        tf = 3;
        param0 = 0;
        paramf = 0;
    }else if (t >= 3 && t < 4){
        t0 = 3;
        tf = 4;
        param0 = 0;
        paramf = 0.5;
    }else if (t >= 4 && t < 5){
        t0 = 4;
        tf = 5;
        param0 = 0.5;
        paramf = 0.5;
    }else if (t >= 5 && t < 9){
        t0 = 5;
        tf = 9;
        param0 = 0.5;
        paramf = 0;
    }

    Eigen::VectorXd YaParameters = calcAParameters(t0, tf, param0, paramf);
    return YaParameters;
}

Eigen::VectorXd getZParameterForT(double t){

    double t0,tf,param0,paramf;
    if (t < 1){
        t0 = 0;
        tf = 1;
        param0 = 1.6;
        paramf = 1;

    }else if (t >= 1 && t < 2){
        t0 = 1;
        tf = 2;
        param0 = 1;
        paramf = 1;
    }else if (t >= 2 && t < 3){
        t0 = 2;
        tf = 3;
        param0 = 1;
        paramf = 1;
    }else if (t >= 3 && t < 4){
        t0 = 3;
        tf = 4;
        param0 = 1;
        paramf = 1;
    }else if (t >= 4 && t < 5){
        t0 = 4;
        tf = 5;
        param0 = 1;
        paramf = 1.6;
    }else if (t >= 5 && t < 9){
        t0 = 5;
        tf = 9;
        param0 = 1.6;
        paramf = 1.6;
    }

    Eigen::VectorXd ZaParameters = calcAParameters(t0, tf, param0, paramf);
    return ZaParameters;
}

Eigen::VectorXd getRZParameterForT(double t){
    double t0,tf,param0,paramf;

    if (t < 1){
        t0 = 0;
        tf = 1;
        param0 = 0;
        paramf = 0;
    }else if (t >= 1 && t < 2){
        t0 = 1;
        tf = 2;
        param0 = 0;
        paramf = M_PI/2;
    }else if (t >= 2 && t < 3){
        t0 = 2;
        tf = 3;
        param0 = M_PI/2;
        paramf = M_PI/2;
    }else if (t >= 3 && t < 4){
        t0 = 3;
        tf = 4;
        param0 = M_PI/2;
        paramf = M_PI/2;
    }else if (t >= 4 && t < 5){
        t0 = 4;
        tf = 5;
        param0 = M_PI/2;
        paramf = 0;
    }else if (t >= 5 && t < 9){
        t0 = 5;
        tf = 9;
        param0 = 0;
        paramf = 0;
    }

    Eigen::VectorXd RZaParameters = calcAParameters(t0, tf, param0, paramf);
    return RZaParameters;
}



Eigen::Affine3d getTargetPosition(double y, double z, double rz){
    Eigen::Affine3d target = Eigen::Translation3d(Eigen::Vector3d(1, y, z))*
                              Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
                              Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitY())*
                              Eigen::AngleAxisd(rz,Eigen::Vector3d::UnitZ());
    return target;
}

std::vector<double> getBestSolution(std::vector<std::vector<double>> solutions){
    //TODO: calculate shortest path
    return solutions[0];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_visualization");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    ROS_INFO("INIT\n\n");

    // Trajectory steps (t,x,y,z,rx,ry,rz)
    std::vector<std::vector<double>> trajectorySteps = {
            {0,1,0,1.6,0,M_PI/2,0}, // T=0
            {1,1,0,1,0,M_PI/2,0}, // T=1
            {2,1,0,1,0,M_PI/2,M_PI/2}, // T=2
            {3,1,0,1,0,M_PI/2,M_PI/2}, // T=3
            {4,1,0.5,1,0,M_PI/2,M_PI/2}, // T=4
            {5,1,0.5,1.6,0,M_PI/2,0}, // T=5
            {9,1,0,1.6,0,M_PI/2,0}, // T=9
    };



    for (double t = 0; t <= 9; t += 0.1) {

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(6);
        point.velocities.resize(6);
        point.accelerations.resize(6);

        // Calc target position with A parameters for t
        double YPosition = getPosition(t,getYParameterForT(t));
        double ZPosition = getPosition(t, getZParameterForT(t));
        double RZRotation = getPosition(t, getRZParameterForT(t));

        Eigen::Affine3d targetPosition = getTargetPosition(YPosition, ZPosition ,RZRotation);

        // Inverse kinematics solver -> get solutions - vypocita hodnoty pre klby
        std::vector<std::vector<double>> solutions = calculateSolutionsForTargetPosition(targetPosition);

        // nájde sa najlepšie riešenie - najkratšia vzdialenosť pohybu
        std::vector<double> bestSolution = getBestSolution(solutions);

        // zapíše sa hodnota pre prve najdene riešenie ako hodnota pre klb
        for (int i = 0; i < 6; ++i) {
            point.positions[i] = bestSolution[i];
            point.velocities[i] = 0;
            point.accelerations[i] = 0;
        }


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


std::vector<std::vector<double>> calculateSolutionsForTargetPosition(Eigen::Affine3d target){

    // Vytvorenie modelu z urdf
    robot_model_loader::RobotModelLoader loader("robot_description");

    // Vyber move group a IK algoritmu
    robot_state::JointModelGroup* joint_model_group = loader.getModel()->getJointModelGroup("robot");
    const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();

    geometry_msgs::Pose target_msg;
    tf::poseEigenToMsg(target, target_msg);

    // Premenne vystupujuce vo funkcii getPositionIK
    std::vector<geometry_msgs::Pose> ik_poses = {target_msg};   // Vlozenie cielu do pola
    std::vector<std::vector<double>> solutions;                // Vystup z funkcie s mnozinou rieseni
    std::vector<double> ik_seed_state = {0,0,0,0,0,0};          // Odhadovane riesenie - potrebne iba pre analyticke algoritmy
    kinematics::KinematicsResult result;                        // exit_code z funkcie (ci bol vypocet uspesny)

    // Vypocet inverznej kinematiky
    solver->getPositionIK(ik_poses,ik_seed_state, solutions, result, kinematics::KinematicsQueryOptions());

    // Overenie ci bol vypocet uspesny
    if (result.kinematic_error != kinematics::KinematicError::OK) {
        throw std::runtime_error("Unable to compute IK. Error: " + std::to_string(result.kinematic_error));
    }

    return solutions;
}