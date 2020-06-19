#ifndef WALL_CONTROLLER_H
#define WALL_CONTROLLER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"

class WallController {
public:
    WallController(ros::NodeHandle &n);

private:
    double time;
    double yaw;
    void odomCallback(const nav_msgs::Odometry &msg);
    void clockCallback(const rosgraph_msgs::Clock &msg);
    ros::NodeHandle n;
    ros::Subscriber subOdom;
    ros::Subscriber subClock;
};

#endif