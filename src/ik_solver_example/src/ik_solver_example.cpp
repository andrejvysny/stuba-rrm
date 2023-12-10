#include <ros/ros.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
    // Inicializacia
    ros::init(argc, argv, "ik_solver_example");
    ros::NodeHandle n;

    // Vytvorenie modelu z urdf
    robot_model_loader::RobotModelLoader loader("robot_description");

    // Vyber move group a IK algoritmu
    robot_state::JointModelGroup* joint_model_group = loader.getModel()->getJointModelGroup("robot");
    const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();


    // Vytvorenie cielovej polohy x = 1.27, y = 0.0, z = 1.0, rx = 0.0, ry = 1.57, rz = 0.0
    Eigen::Affine3d target = Eigen::Translation3d(Eigen::Vector3d(1.27, 0, 1.0))*
                           Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*
                           Eigen::AngleAxisd(1.57,Eigen::Vector3d::UnitY())*
                           Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ());

    // Konverzia z Eigen do geometry_msgs
    geometry_msgs::Pose target_msg;
    tf::poseEigenToMsg(target, target_msg);

    // Premenne vystupujuce vo funkcii getPositionIK
    std::vector<geometry_msgs::Pose> ik_poses = {target_msg};   // Vlozenie cielu do pola
    std::vector<std::vector<double> > solutions;                // Vystup z funkcie s mnozinou rieseni
    std::vector<double> ik_seed_state = {0,0,0,0,0,0};          // Odhadovane riesenie - potrebne iba pre analyticke algoritmy
    kinematics::KinematicsResult result;                        // exit_code z funkcie (ci bol vypocet uspesny)

    // Vypocet inverznej kinematiky
    solver->getPositionIK(ik_poses,ik_seed_state, solutions, result, kinematics::KinematicsQueryOptions());

    // Overenie ci bol vypocet uspesny
    if (result.kinematic_error != kinematics::KinematicError::OK) {
        throw std::runtime_error("Unable to compute IK. Error: " + std::to_string(result.kinematic_error));
    }

    // Vypis riesenii
    for (const auto &solution: solutions) {
        ROS_INFO("Solution found: ");
        for (const auto &joint : solution) {
            ROS_INFO_STREAM(std::to_string(joint));
        }
    }
    return 0;
}