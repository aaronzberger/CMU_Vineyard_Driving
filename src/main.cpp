#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"



int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_controller");

    ros::NodeHandle n;

    ros::spin();

    return 0;
}