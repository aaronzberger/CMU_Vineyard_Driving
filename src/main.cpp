#include <numeric>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include "pid.h"
#include "pid.cpp"
#include <opencv2/core/mat.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <png++/png.hpp>



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg);
bool isWallPoint(const pcl::PointXYZ &pt);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void velodyneCallBack(const PointCloud::ConstPtr &msg);
double ransac(const PointCloud::ConstPtr &pcl);


double currentTime, yaw, lastMotionUpdate;
ros::Publisher velPub;
PID controller;

/**
 * @brief Updates the current time variable
 * 
 * @param msg the message received from the clock ros topic
 */
void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg) {
    currentTime = (msg->clock.sec * 1000) + int(msg->clock.nsec / 1e6);
}

/**
 * @brief Whether the point is a point belonging to a wall
 * 
 * @param pt the point in the point cloud, containing an X, Y, and Z
 * @return a boolean representing whether the point is on a wall
 */
bool isWallPoint(const pcl::PointXYZ &pt) {
    return (!(pt.z > -0.375 && pt.z < -0.365));
}

/**
 * @brief Update the yaw variable
 * 
 * @param msg the message received from the odometry ros topic
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    tf::Quaternion quat;
    quat.setW(msg->pose.pose.orientation.w);
    quat.setX(msg->pose.pose.orientation.x);
    quat.setY(msg->pose.pose.orientation.y);
    quat.setZ(msg->pose.pose.orientation.z);

    yaw = tf::getYaw(quat);
}

/**
 * @brief Update the output to the robot using Lidar data, through a PID controller
 * 
 * @param msg the message received from the Lidar ros topic (velodyne)
 */
void velodyneCallBack(const PointCloud::ConstPtr &msg) {
    if(currentTime - lastMotionUpdate > 10) {
        // Simple Linear Regression, using all points

        // double slope, yInt, sumX, sumY, sumXY, sumX2;

        // int numPoints {0};
        // for(int i{0}; i < msg->width - 0; i++) {
        //     if(isWallPoint(msg->points.at(i))) {
        //         sumX += msg->points.at(i).x;
        //         sumY += msg->points.at(i).y;
        //         sumXY += msg->points.at(i).x * msg->points.at(i).y;
        //         sumX2 += msg->points.at(i).x * msg->points.at(i).x;
        //         numPoints++;
        //     }
        // }
        // double xMean = sumX / numPoints;
        // double yMean = sumY / numPoints;
        // double denominator = sumX2 - sumX * xMean;
        // slope = (sumXY - sumX * yMean) / denominator;
        // yInt = yMean - slope * xMean;


        double slope {ransac(msg)};

        double angle {std::atan(slope)};

        std::cout << "Relative Wall Angle: " << angle * (180 / 3.1415)  << std::endl;

        double angVel {controller.calculate(angle, currentTime)};

        geometry_msgs::Twist twist;

        twist.linear.x = 0.2;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = angVel;

        velPub.publish(twist);
        lastMotionUpdate = currentTime;

        // for(int i {0}; i < msg->width; i++) {
        //     std::cout << "(" << msg->points.at(i).x << ", " << msg->points.at(i).y << ", " << msg->points.at(i).z << ")" << std::endl;
        // }
    }
}

double ransac(const PointCloud::ConstPtr &pcl) {
    double inlierThreshold {0.1};
    int numLoops {250};

    std::vector<double> bestFit {0, 0};
    double yint {0};
    // std::cout << "Points: " << pcl->width << std::endl;
    for(int i{0}; i < numLoops; i++) {
        int pt1Idx {std::rand() / ((RAND_MAX + 1u) / pcl->width)};
        while(pcl->points.at(pt1Idx).z < 0.5) {
            pt1Idx = std::rand() / ((RAND_MAX + 1u) / pcl->width);
        }
        int pt2Idx {std::rand() / ((RAND_MAX + 1u) / pcl->width)};
        while(pcl->points.at(pt2Idx).z < 0.5 || pt2Idx == pt1Idx) {
            pt2Idx = std::rand() / ((RAND_MAX + 1u) / pcl->width);
        }

        double a {pcl->points.at(pt2Idx).y - pcl->points.at(pt1Idx).y};
        double b {pcl->points.at(pt1Idx).x - pcl->points.at(pt2Idx).x};
        double c {(a * pcl->points.at(pt1Idx).x) + (b * pcl->points.at(pt1Idx).y)};
        int inliers {0};
        // std::cout << "X1: " << pcl->points.at(pt1Idx).x << "Y1: " << pcl->points.at(pt1Idx).y <<
        //              "X2: " << pcl->points.at(pt2Idx).x << "Y2: " << pcl->points.at(pt2Idx).y << std::endl;
        // std::cout << "A: " << a << ", B: " << b << ", C: " << c << std::endl;
        // std::cout << "Slope: " << -(a / b) << ", Yint: " << c / b << std::endl;
        for(int j{0}; j < pcl->width; j++) {
            double distance {(std::abs((a * pcl->points.at(j).x) + (b * pcl->points.at(j).y) - c)) / (std::sqrt(a*a + b*b))};
            if(distance < inlierThreshold) inliers++;
        }
        // std::cout << "Inliers: " << inliers << std::endl;
        if(inliers > bestFit.at(1)) {
            bestFit.at(0) = -(a / b);
            bestFit.at(1) = inliers;
            yint = c / b;
        }
    }

    // For testing, print valid points to a csv file for graphing

    // std::cout << "Final Slope: " << bestFit.at(0) << "Final Yint" << yint << std::endl;
    // std::cout << "Inliers: " << bestFit.at(1) << std::endl;
    // std::cout << "--------------" << std::endl;
    // std::ofstream graphing("lineGraphing.csv");
    // for(int i{0}; i < pcl->width; i++) {
    //     if(pcl->points.at(i).z > 0.5) {
    //         graphing << pcl->points.at(i).x << "," << pcl->points.at(i).y << "," << pcl->points.at(i).z << "\n";
    //     }
    // }
    // graphing.close();

    return bestFit.at(0);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_controller");

    if(argc != 4) {
        std::cout << "Expected 3 arguments: P, I, D" << std::endl;
        return 0;
    }

    ros::NodeHandle n;

    std::srand(std::time(nullptr));

    ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, odomCallback);
    ros::Subscriber subClock = n.subscribe<rosgraph_msgs::Clock>("/clock", 1000, clockCallback);
    ros::Subscriber subVelodyne = n.subscribe<PointCloud>("/velodyne_points", 1000, velodyneCallBack);
    velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    controller = PID(std::atof(argv[0]), std::atof(argv[1]), std::atof(argv[2]));

    controller.setInverted(false);
    controller.setSetPoint(0);

    ros::spin();

    return 0;
}

