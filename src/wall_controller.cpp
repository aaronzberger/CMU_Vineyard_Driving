#include "wall_controller.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"

WallController::WallController(ros::NodeHandle &n) 
    : n{n} {
    subOdom = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, odomCallback);
    subClock = n.subscribe<rosgraph_msgs::Clock>("/clock", 1000, clockCallback);
}

void WallController::odomCallback(const nav_msgs::Odometry &msg) {
    tf::Quaternion quat;
    quat.setW(msg.pose.pose.orientation.w);
    quat.setX(msg.pose.pose.orientation.x);
    quat.setY(msg.pose.pose.orientation.y);
    quat.setZ(msg.pose.pose.orientation.z);

    this->yaw = tf::getYaw(quat);
}

void WallController::clockCallback(const rosgraph_msgs::Clock &msg) {
    this->time = msg.clock.sec * 1000 + int(msg.clock.nsec / 1e6);
}