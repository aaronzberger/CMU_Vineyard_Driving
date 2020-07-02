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

struct line {
    double a, b, c, distance;
    unsigned inliers;
};

struct lineGroup {
    std::vector<line> lines;
    int totalInliers;
};

//Function Prototypes
void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg);
bool isGroundPoint(const pcl::PointXYZ &pt);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void velodyneCallBack(const PointCloud::ConstPtr &msg);
lineGroup ransac(const PointCloud::ConstPtr &pcl);
bool close(std::vector<double> line1, std::vector<double> line2);
double lineToPtDistance(double x, double y, double a, double b, double c);
void moveLineVecsBack();
void moveVinePtsVecBack();

double currentTime, yaw, x, y, lastMotionUpdate;
ros::Publisher velPub, markerPub;
PID controller;
int counter, markerID;

//Storing the past lines for a moving average
std::vector<std::vector<double>> tMinus2Lines;
std::vector<std::vector<double>> tMinus1Lines;
std::vector<std::vector<double>> tZeroLines;
std::vector<std::vector<double>> tPlus1Lines;
std::vector<std::vector<double>> tPlus2Lines;

//Storing the past number of valid points for a moving average
std::vector<int> numVinePtsVec(49, -1);

std::ofstream vinePtsCounterFile("vinePointsCounter.csv");

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
 * @brief Move back the past lines to make room for a new one every frame
 */
void moveLineVecsBack() {
    tMinus2Lines = tMinus1Lines;
    tMinus1Lines = tZeroLines;
    tZeroLines = tPlus1Lines;
    tPlus1Lines = tPlus2Lines;
}

/**
 * @brief Move back the past valid points to make room for a new value every frame
 */
void moveVinePtsVecBack() {
    int numPts = numVinePtsVec.size();
    for(int i{1}; i < numPts; i++) {
        numVinePtsVec.at(i - 1) = numVinePtsVec.at(i);
    }
}

void displayLine(line line, float r, float g, float b) {
    uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();

    marker.ns = "shapes";
    marker.id = markerID;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point pt1;
    pt1.x = -100;
    // pt1.y = (lines.at(0).at(0) * pt1.x) + lines.at(0).at(1);
    pt1.y = ((-(line.a / line.b)) * pt1.x) + (line.c / line.b);
    pt1.z = 0;

    geometry_msgs::Point pt2;
    pt2.x = 100;
    // pt2.y = (lines.at(0).at(0) * pt2.x) + lines.at(0).at(1);
    pt2.y = ((-(line.a / line.b)) * pt2.x) + (line.c / line.b);
    pt2.z = 0;

    marker.points.push_back(pt1);
    marker.points.push_back(pt2);

    marker.scale.x = 0.15;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    markerPub.publish(marker);
}

/**
 * @brief Update the output to the robot using Lidar data, through a PID controller
 * 
 * @param msg the message received from the Lidar ros topic (velodyne)
 */
void velodyneCallBack(const PointCloud::ConstPtr &msg) {
    if(currentTime - lastMotionUpdate > 10) {
        // Collect the detected lines from the RANSAC algorithm
        lineGroup lines {ransac(msg)};

        // For testing, write a file with the contents of all the valid points in the most recent point cloud
        int numVinePts {0};
        std::ofstream vinePointsFile("vinePoints.csv");
        for(int i{0}; i < msg->width; i++) {
            if(!isGroundPoint(pcl->points.at(i))) {
                vinePointsFile << msg->points.at(i).x << "," << msg->points.at(i).y << "," << msg->points.at(i).z << "\n";
                numVinePts++;
            }
        }
        vinePointsFile.close();
        vinePtsCounterFile << counter << "," << numVinePts;

        // // Detect the probable end of a row by keeping track of the number of vine points found
        // moveVinePtsVecBack();
        // numVinePtsVec.at(numVinePtsVec.size() - 1) = numVinePts;

        // if(!(numVinePtsVec.at(0) == -1)) {
        //     double movingAverage {0};
        //     for(auto val : numVinePtsVec) movingAverage += val;
        //     movingAverage /= numVinePtsVec.size();
        //     vinePtsCounterFile << "," << movingAverage;
        // }
        // counter++;
        // vinePtsCounterFile << "\n";

        //Store the last 5 entries for a moving average smoothing filter
        moveLineVecsBack();
        tPlus2Lines = lines;

        if(!tMinus2Lines.empty()) {
            //Calculate the average of the last 5 lines
            double leftLineSlope {(tMinus2Lines.at(0).at(0) + tMinus1Lines.at(0).at(0) + tZeroLines.at(0).at(0) + tPlus1Lines.at(0).at(0) + tPlus2Lines.at(0).at(0)) / 5.0};
            double leftLineYint {(tMinus2Lines.at(0).at(1) + tMinus1Lines.at(0).at(1) + tZeroLines.at(0).at(1) + tPlus1Lines.at(0).at(1) + tPlus2Lines.at(0).at(1)) / 5.0};
            double rightLineSlope {(tMinus2Lines.at(1).at(0) + tMinus1Lines.at(1).at(0) + tZeroLines.at(1).at(0) + tPlus1Lines.at(1).at(0) + tPlus2Lines.at(1).at(0)) / 5.0};
            double rightLineYint {(tMinus2Lines.at(1).at(1) + tMinus1Lines.at(1).at(1) + tZeroLines.at(1).at(1) + tPlus1Lines.at(1).at(1) + tPlus2Lines.at(1).at(1)) / 5.0};

            //DISPLAY DETECTED LINES IN RVIZ
            //******************************************************//
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
            // pt1.y = (lines.at(0).at(0) * pt1.x) + lines.at(0).at(1);
            pt1.y = (leftLineSlope * pt1.x) + leftLineYint;
            pt1.z = 0;

            geometry_msgs::Point pt2;
            pt2.x = 100;
            // pt2.y = (lines.at(0).at(0) * pt2.x) + lines.at(0).at(1);
            pt2.y = (leftLineSlope * pt2.x) + leftLineYint;
            pt2.z = 0;

            marker1.points.push_back(pt1);
            marker1.points.push_back(pt2);

            marker1.scale.x = 0.15;
    
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
            // pt3.y = (lines.at(1).at(0) * pt3.x) + lines.at(1).at(1);
            pt3.y = (rightLineSlope * pt3.x) + rightLineYint;
            pt3.z = 0;

            geometry_msgs::Point pt4;
            pt4.x = 100;
            // pt4.y = (lines.at(1).at(0) * pt4.x) + lines.at(1).at(1);
            pt4.y = (rightLineSlope * pt4.x) + rightLineYint;
            pt4.z = 0;

            marker2.points.push_back(pt3);
            marker2.points.push_back(pt4);

            marker2.scale.x = 0.15;
    
            // Set the color -- be sure to set alpha to something non-zero!
            marker2.color.r = 1.0f;
            marker2.color.g = 0.0f;
            marker2.color.b = 0.0f;
            marker2.color.a = 1.0;
    
            marker2.lifetime = ros::Duration();
            markerPub.publish(marker2);
            //******************************************************//
        
            //Calculate the input for the PID controller
            double angle {std::atan(rightLineSlope)};

            //Run the PID controller
            double angVel {controller.calculate(angle, currentTime)};

            //Construct a velocity message to give to the Husky
            geometry_msgs::Twist twist;

            twist.linear.x = 0.3;
            twist.linear.y = 0;
            twist.linear.z = 0;

            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0.0;

            velPub.publish(twist);
            lastMotionUpdate = currentTime;
        }
    }
}

lineGroup ransac(const PointCloud::ConstPtr &pcl) {
    double inlierThreshold {0.3};
    int numLoops {500};
    int minInlierThreshold {20};
    double distanceBetweenLines {3.5};

    lineGroup bestLineGroup;
    bestLineGroup.totalInliers = 0;

    for(int i{0}; i < numLoops; i++) {
        // Find two random points for RANSAC line detection
        int pt1Idx {std::rand() / ((RAND_MAX) / (pcl->width - 1))};
        while(isGroundPoint(pcl->points.at(pt1Idx))) {
            pt1Idx = std::rand() / ((RAND_MAX) / (pcl->width - 1));
        }
        int pt2Idx {std::rand() / ((RAND_MAX) / (pcl->width - 1))};
        while(isGroundPoint(pcl->points.at(pt2Idx)) || pt2Idx == pt1Idx) {
            pt2Idx = std::rand() / ((RAND_MAX) / (pcl->width - 1));
        }

        // Construct a line from those two random points
        double a {pcl->points.at(pt2Idx).y - pcl->points.at(pt1Idx).y};
        double b {pcl->points.at(pt1Idx).x - pcl->points.at(pt2Idx).x};
        double c {(a * pcl->points.at(pt1Idx).x) + (b * pcl->points.at(pt1Idx).y)};

        // Calculate how many inliers that line has
        // for(int j{0}; j < pcl->width; j++) {
        //     if(!isGroundPoint(pcl->points.at(j))) {
        //         double distance {lineToPtDistance(pcl->points.at(j).x, pcl->points.at(j).y, a, b, c)};
        //         if(std::abs(distance) < inlierThreshold) {
        //             inliers++;
        //         }
        //     }
        // }

        // Determine the number of inliers in the originally detected line
        int inliers {0};
        for(int j{0}; i < pcl->width; j++) {
            if(!isGroundPoint(pcl->points.at(j))) {
                double distance {lineToPtDistance(pcl->points.at(j).x, pcl->points.at(j).y, a, b, c)};
                if(std::abs(distance) < inlierThreshold) inliers++;
            }
        }

        // If the original line is strong enough, look around for other parallel vine rows
        if(inliers > minInlierThreshold) {
            lineGroup lineGroup;
            lineGroup.totalInliers = 0;

            int lookaroundRadius{2};

            for(int i{-lookaroundRadius}; i <= lookaroundRadius; i++) {
                line line;
                line.a = a;
                line.b = b;
                line.c = c + (i * distanceBetweenLines);
                line.distance = lineToPtDistance(x, y, line.a, line.b, line.c);
                line.inliers = 0;
                for(int j{0}; i < pcl->width; j++) {
                    if(!isGroundPoint(pcl->points.at(j))) {
                        double distance {lineToPtDistance(pcl->points.at(j).x, pcl->points.at(j).y, line.a, line.b, line.c)};
                        if(std::abs(distance) < inlierThreshold) line.inliers++;
                    }
                }
                lineGroup.totalInliers += line.inliers;
                lineGroup.lines.push_back(line);
            }
        }

        if(lineGroup.totalInliers > bestLineGroup.totalInliers) bestLineGroup = lineGroup;
    }
    return bestLineGroup;
}

/**
 * @brief Determine if two lines are too close to be different lines
 * 
 * @param line1 the first line
 * @param line2 the second line
 * 
 * @return whether the two lines represent the same detected line
 */
bool close(std::vector<double> line1, std::vector<double> line2) {
    if(std::abs(line1.at(0) - line2.at(0)) < 0.2) {
        return (std::abs(line1.at(1) - line2.at(1)) < 0.3);
    }
    return false;
}

/**
 * @brief Determine the distance from a point to a line in standard form
 * 
 * @param x the x of the point
 * @param y the y of the point
 * @param a the a of the line
 * @param b the b of the line
 * @param c the c of the line
 * 
 * @return the distance from the point to the line in standard form
 */
double lineToPtDistance(double x, double y, double a, double b, double c) {
    return (((a * x) + (b * y) - c) / (std::sqrt(a*a + b*b)));
}

/**
 * @brief Print the contents of a point cloud for testing
 * 
 * @param pcl the point cloud to print
 */
void printPointCloud(const PointCloud::ConstPtr &pcl) {
    for(int i{0}; i < pcl->width; i++) {
        if(!isGroundPoint(pcl->points.at(i))) {
            std::cout << "X: " << pcl->points.at(i).x << " Y: " << pcl->points.at(i).y << " Z: " << pcl->points.at(i).z << std::endl;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_controller");

    if(argc != 4) {
        std::cout << "Expected 3 arguments: P, I, D" << std::endl;
        return -1;  // by convention, return a non-zero code to indicate error
    }

    counter = 0;
    markerID = 0;

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

    vinePtsCounterFile.close();

    return 0;
}

