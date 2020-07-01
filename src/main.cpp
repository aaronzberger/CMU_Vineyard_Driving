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
bool isGroundPoint(const pcl::PointXYZ &pt);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void velodyneCallBack(const PointCloud::ConstPtr &msg);
std::vector<std::vector<double>> ransac(const PointCloud::ConstPtr &pcl);
bool close(std::vector<double> line1, std::vector<double> line2);
double lineToPtDistance(double x, double y, double a, double b, double c);


double currentTime, yaw, x, y, lastMotionUpdate;
ros::Publisher velPub, markerPub;
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
bool isGroundPoint(const pcl::PointXYZ &pt) {
    return (pt.z > -0.375 && pt.z < -0.365);
}

/**
 * @brief Update the yaw variable
 * 
 * @param msg the message received from the odometry ros topic
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

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
        std::vector<std::vector<double>> lines {ransac(msg)};

        double bestLineSlope {lines.at(0).at(2) > lines.at(1).at(2) ? lines.at(0).at(0) : lines.at(1).at(0)};

        // double distanceDiff {std::abs(lines.at(0).at(3) - lines.at(1).at(3))};
        // std::cout << "Distance diff: " << distanceDiff << std::endl;

        uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

        visualization_msgs::Marker marker1;
        marker1.header.frame_id = "odom";
        marker1.header.stamp = ros::Time::now();

        marker1.ns = "shapes";
        marker1.id = 0;
        marker1.type = shape;
        marker1.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point pt1;
        pt1.x = -100;
        pt1.y = (lines.at(0).at(0) * pt1.x) + lines.at(0).at(1);
        pt1.z = 0;

        geometry_msgs::Point pt2;
        pt2.x = 100;
        pt2.y = (lines.at(0).at(0) * pt2.x) + lines.at(0).at(1);
        pt2.z = 0;

        marker1.points.push_back(pt1);
        marker1.points.push_back(pt2);

        marker1.scale.x = 0.2;
   
        // Set the color -- be sure to set alpha to something non-zero!
        marker1.color.r = 0.0f;
        marker1.color.g = 1.0f;
        marker1.color.b = 0.0f;
        marker1.color.a = 1.0;
   
        marker1.lifetime = ros::Duration();
        markerPub.publish(marker1);


        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "odom";
        marker2.header.stamp = ros::Time::now();

        marker2.ns = "shapes";
        marker2.id = 1;
        marker2.type = shape;
        marker2.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point pt3;
        pt3.x = -100;
        pt3.y = (lines.at(1).at(0) * pt3.x) + lines.at(1).at(1);
        pt3.z = 0;

        geometry_msgs::Point pt4;
        pt4.x = 100;
        pt4.y = (lines.at(1).at(0) * pt4.x) + lines.at(1).at(1);
        pt4.z = 0;

        marker2.points.push_back(pt3);
        marker2.points.push_back(pt4);

        marker2.scale.x = 0.2;
   
        // Set the color -- be sure to set alpha to something non-zero!
        marker2.color.r = 0.0f;
        marker2.color.g = 1.0f;
        marker2.color.b = 0.0f;
        marker2.color.a = 1.0;
   
        marker2.lifetime = ros::Duration();
        markerPub.publish(marker2);

        double angle {std::atan(bestLineSlope)};

        //std::cout << "Theta: " << angle * (180 / 3.1415)  << std::endl;

        double angVel {controller.calculate(angle, currentTime)};

        //std::cout << "PID Output: " << angVel << std::endl;

        geometry_msgs::Twist twist;

        twist.linear.x = 0.0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0.0;

        velPub.publish(twist);
        lastMotionUpdate = currentTime;
    }
}

std::vector<std::vector<double>> ransac(const PointCloud::ConstPtr &pcl) {
    double inlierThreshold {0.3};
    int numLoops {500};

    // Print the entire Point Cloud
    // for(int i{0}; i < pcl->width; i++) {
    //     std::cout << "X: " << pcl->points.at(i).x << " Y: " << pcl->points.at(i).y << " Z: " << pcl->points.at(i).z << std::endl;
    // }

    //Declare a vector that stores the {slope, yIntercept, inliers} for the best two lines
    std::vector<std::vector<double>> bestFits { {0, 0, 0, 0}, {0, 0, 0, 0} };
    // std::vector<std::vector<double>> inliers1;
    // std::vector<std::vector<double>> inliers2;

    double yint {0};
    for(int i{0}; i < numLoops; i++) {
        // std::vector<std::vector<double>> inlierVec;
        //inlierVec.clear();
        int pt1Idx {std::rand() / ((RAND_MAX) / (pcl->width - 1))};
        while(isGroundPoint(pcl->points.at(pt1Idx))) {
            pt1Idx = std::rand() / ((RAND_MAX) / (pcl->width - 1));
        }
        int pt2Idx {std::rand() / ((RAND_MAX) / (pcl->width - 1))};
        while(isGroundPoint(pcl->points.at(pt2Idx)) || pt2Idx == pt1Idx) {
            pt2Idx = std::rand() / ((RAND_MAX) / (pcl->width - 1));
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
            if(!isGroundPoint(pcl->points.at(j))) {
                double distance {lineToPtDistance(pcl->points.at(j).x, pcl->points.at(j).y, a, b, c)};
                if(std::abs(distance) < inlierThreshold) {
                    inliers++;
                    // std::vector<double> instance {pcl->points.at(j).x, pcl->points.at(j).y, pcl->points.at(j).z};
                    // inlierVec.push_back(instance);
                    //std::cout << "Inlier: x: " << pcl->points.at(j).x << " y: " << pcl->points.at(j).y << " z: " << pcl->points.at(j).z << std::endl;
                }
            }
        }

        // std::cout << "Best lines: Slope1: " << bestFits.at(0).at(0) << " Yint1: " << bestFits.at(0).at(1) << " Slope2: " << bestFits.at(1).at(0) << " Yint2: " << bestFits.at(1).at(1) << std::endl;
        
        // To represent the line, {slope, yInt, inliers, distance to line}
        double distanceToLine {lineToPtDistance(x, y, a, b, c)};
        std::vector<double> newLine {(-(a/b)), (c/b), static_cast<double>(inliers), distanceToLine};

        if(close(bestFits.at(0), newLine) && inliers > bestFits.at(0).at(2)) {
            bestFits.at(0) = newLine;
            // inliers1 = inlierVec;
        } else if(close(bestFits.at(1), newLine) && inliers > bestFits.at(1).at(2)) {
            bestFits.at(1) = newLine;
            // inliers2 = inlierVec;
        } else {
            size_t worstLineIdx {bestFits.at(0).at(2) > bestFits.at(1).at(2) ? 1 : 0};
            if(inliers > bestFits.at(worstLineIdx).at(2) && !close(bestFits.at(1 - worstLineIdx), newLine)) {
                bestFits.at(worstLineIdx) = newLine;
                // if(worstLineIdx == 0) inliers1 = inlierVec;
                // else inliers2 = inlierVec;
            }
        }

        // if(close(bestFits.at(0), newLine)) {
        //     if(inliers > bestFits.at(0).at(2)) {
        //         bestFits.at(0) = newLine;
        //     }
        // }
    }

    // For testing, print valid points to a csv file for graphing
    std::cout << "--------------" << std::endl;
    std::cout << "Final Slope: " << bestFits.at(0).at(0) << " Final Yint" << bestFits.at(0).at(1) << std::endl;
    std::cout << "Inliers: " << bestFits.at(0).at(2) << " Distance: " << bestFits.at(0).at(3) << std::endl;
    // for(auto vec: inliers1) {
    //     std::cout << "Inlier: x: " << vec.at(0) << " y: " << vec.at(1) << " z: " << vec.at(2) << std::endl;
    // }

    std::cout << "Final Slope: " << bestFits.at(1).at(0) << " Final Yint" << bestFits.at(1).at(1) << std::endl;
    std::cout << "Inliers: " << bestFits.at(1).at(2) << " Distance: " << bestFits.at(1).at(3) << std::endl;
    // for(auto vec: inliers2) {
    //     std::cout << "Inlier: x: " << vec.at(0) << " y: " << vec.at(1) << " z: " << vec.at(2) << std::endl;
    // }
    std::cout << "--------------" << std::endl;
    std::ofstream graphing("lineGraphing.csv");
    for(int i{0}; i < pcl->width; i++) {
        if(!isGroundPoint(pcl->points.at(i))) {
            graphing << pcl->points.at(i).x << "," << pcl->points.at(i).y << "," << pcl->points.at(i).z << "\n";
        }
    }
    graphing.close();

    return bestFits;
}

bool close(std::vector<double> line1, std::vector<double> line2) {
    // std::cout << line1.at(0) << ", " << line1.at(1) << "; " << line2.at(0) << ", " << line2.at(1) << " : ";
    // if(std::abs(line1.at(0) - line2.at(0)) < 0.2) {
    //     std::cout << (std::abs(line1.at(1) - line2.at(1)) < 0.3) << std::endl;
    // }
    // else std::cout << false << std::endl;
    if(std::abs(line1.at(0) - line2.at(0)) < 0.2) {
        return (std::abs(line1.at(1) - line2.at(1)) < 0.3);
    }
    return false;
}

double lineToPtDistance(double x, double y, double a, double b, double c) {
    return (((a * x) + (b * y) - c) / (std::sqrt(a*a + b*b)));
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_controller");

    if(argc != 4) {
        std::cout << "Expected 3 arguments: P, I, D" << std::endl;
        return -1;  // by convention, return a non-zero code to indicate error
    }

    std::cout << std::boolalpha;

    ros::NodeHandle n;

    std::srand(std::time(nullptr));

    ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, odomCallback);
    ros::Subscriber subClock = n.subscribe<rosgraph_msgs::Clock>("/clock", 1000, clockCallback);
    ros::Subscriber subVelodyne = n.subscribe<PointCloud>("/velodyne_points", 1000, velodyneCallBack);
    velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    markerPub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    controller = PID(std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]));

    controller.setInverted(false);
    controller.setSetPoint(0);
    controller.setOutputLimits(-0.4, 0.4);
    controller.setMaxIOutput(0.2);

    ros::spin();

    return 0;
}

