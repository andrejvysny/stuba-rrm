#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <stdio.h>

std::vector<double> generateTrajectory(){

    std::vector<double>[] trajectory;

    for (double t=0.0; t<=4.0; t += 0.01){
        
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;


    ros::Publisher publisher = n.advertise<std_msgs::Float64>("/trajectory", 1);


    moveit_msgs::DisplayTrajectory displayTrajectory;
    displayTrajectory.model_id = "abb_irb";
    displayTrajectory.trajectory.push_back(
            generateTrajectory()
            );

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        publisher.publish(displayTrajectory);
        loop_rate.sleep();
        ROS_INFO("Published Trajectory")
    }

    return 0;
}
