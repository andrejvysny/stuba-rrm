#include <ros/ros.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <math.h>


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


std::vector<Eigen::Affine3d> generateTargetPositions(){

    ROS_INFO("Generating targets: start");
    std::vector<Eigen::Affine3d> targets;



//    // Vytvorenie cielovej polohy x = 1.27, y = 0.0, z = 1.0, rx = 0.0, ry = 1.57, rz = 0.0
//    Eigen::Affine3d target = Eigen::Translation3d(Eigen::Vector3d(1.27, 0, 1.0))*
//                             Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
//                             Eigen::AngleAxisd(1.57,Eigen::Vector3d::UnitY())*
//                             Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ());
//    targets.push_back(target);


    // T=0
    Eigen::Affine3d target0 = Eigen::Translation3d(Eigen::Vector3d(1, 0, 1.6))*
                             Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
                             Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitY())*
                             Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ());
    targets.push_back(target0);


    // T=1
    Eigen::Affine3d target1 = Eigen::Translation3d(Eigen::Vector3d(1, 0, 1))*
                             Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
                             Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitY())*
                             Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ());
    targets.push_back(target1);


    // T=2
    Eigen::Affine3d target2 = Eigen::Translation3d(Eigen::Vector3d(1, 0, 1))*
                              Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
                              Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitY())*
                              Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ());
    targets.push_back(target2);


    // T=3
    Eigen::Affine3d target3 = Eigen::Translation3d(Eigen::Vector3d(1, 0, 1))*
                              Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
                              Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitY())*
                              Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ());
    targets.push_back(target3);


    // T=4
    Eigen::Affine3d target4 = Eigen::Translation3d(Eigen::Vector3d(1, 0.5, 1))*
                              Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
                              Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitY())*
                              Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ());
    targets.push_back(target4);


    // T=5
    Eigen::Affine3d target5 = Eigen::Translation3d(Eigen::Vector3d(1, 0.5, 1.6))*
                              Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
                              Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitY())*
                              Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ());
    targets.push_back(target5);


    // T=9
    Eigen::Affine3d target6 = Eigen::Translation3d(Eigen::Vector3d(1, 0, 1.6))*
                              Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
                              Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitY())*
                              Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ());
    targets.push_back(target6);


    ROS_INFO("Generating targets: end");
    return targets;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "iksolver");
    ros::NodeHandle n;
    ROS_INFO("INIT");

    std::vector<Eigen::Affine3d> targets;

    targets = generateTargetPositions();

    ROS_INFO("Targets generated: %ld", targets.size());


    int targetId = 0;
    for(const auto &target: targets){
        ROS_INFO("\n\n\n########## CALCULATING FOR TARGET %d ##########\n", targetId);

        for (const auto &solution: calculateSolutionsForTargetPosition(target)) {
            ROS_INFO("\n\nSolution found: ");
            for (const auto &joint : solution) {
                ROS_INFO_STREAM(std::to_string(joint));
            }
        }
        ROS_INFO("\n\n########## END OF SOLUTIONS FOR TARGET %d ##########\n\n", targetId);

        targetId++;

    }

    return 0;
}