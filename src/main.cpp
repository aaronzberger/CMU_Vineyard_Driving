#include <numeric>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <algorithm>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "visualization_msgs/Marker.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include "pid.h"
#include "pid.cpp"
#include "kalman.h"
#include "kalman.cpp"
#include <Eigen/Dense>

// USER PARAMETERS
constexpr double distanceBetweenRows = 3.0;
constexpr double inlierThreshold = 0.39;
constexpr unsigned numRansacLoops = 250;

constexpr double turningSpeed = 0.4;
constexpr double drivingSpeed = 0.2;

constexpr unsigned numInitialStateDetections = 5;

constexpr double maxPIDOutput = 0.4;
constexpr double maxIOutput = 0.2;

constexpr double initialCovarianceInput[2][2] = {{0.3, 0  },
                                                 {0  , 0.3}};
constexpr double initialModelErrorInput[2][2] = {{0.02, 0},
                                                 {0, 0.02}};
constexpr double initialMeasurementErrorInput[2][2] = {{0.25, 0   },
                                                       {0   , 0.25}};

constexpr double groundPointThreshold = 0.05;

// Proportion of points that are ahead of the robot
constexpr double endOfRowRatio = 0.2;
constexpr double startOfRowRatio = 0.75;

// Number of times the above threshold is met before switching modes
constexpr int endOfRowThreshold = 20;
constexpr int startOfRowThreshold = 20;

constexpr double maxTimeBetweenVeloUpdates = 1.5;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct line {
    double a, b, c, distance, theta;
    unsigned inliers;
};

struct lineGroup {
    std::vector<line> lines;
    int totalInliers;
};

// Function Prototypes
bool isGroundPoint(const pcl::PointXYZ &pt);
double getYaw(double w, double x, double y, double z);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void displayLine(line line, float r, float g, float b, int id);
double getSlope(line line);
double getYInt(line line);
double lineToPtDistance(double x, double y, line l);
double lineToPtAngle(double x, double y, line l);
line polarToStandard(double distance, double theta);
void velodyneCallBack(const PointCloud::ConstPtr &pcl);
lineGroup ransac(const PointCloud::ConstPtr &pcl);

// Robot odometry
double yaw, x, y;

ros::Time lastMotionUpdate;
ros::Publisher markerPub;

PID controller;
Kalman filter;

int endOfRowCounter;
int startOfRowCounter;
bool turning;

// So the robot knows which way to go down the row (true until robot turns into the second row)
bool directionForward;

// For csv writing
std::ofstream kalmanGraphing;
int counter;

bool firstLidarCallback;

// For determining the current odometry in a local frame, the last global frame must be stored
Eigen::Matrix3d Tglobal_lastFrame;

line groundTruthLine;

// Globally store the current message and update it from within every method so frequency of publishing is constant
geometry_msgs::Twist currentVelocityMessage;
ros::Time lastVelocityUpdate;


/**
 * @brief Whether the point is a point not belonging to a wall
 * 
 * @param pt the point in the point cloud, containing an X, Y, and Z
 * @return a boolean representing whether the point is not on a wall
 */
bool isGroundPoint(const pcl::PointXYZ &pt) {
    return pt.z < groundPointThreshold;
}

/**
 * @brief Find the yaw from a quaternion
 * 
 * @param w the w of the quaternion
 * @param x the x of the quaternion
 * @param y the y of the quaternion
 * @param z the z of the quaternion
 * 
 * @return the yaw calculated
 */
double getYaw(double w, double x, double y, double z) {
    tf::Quaternion quat;
    quat.setW(w);
    quat.setX(x);
    quat.setY(y);
    quat.setZ(z);

    return tf::getYaw(quat);
}

/**
 * @brief Update the x, y, and yaw global variables
 * 
 * @param msg the message received from the odometry ros topic
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    yaw = getYaw(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}

/**
 * @brief display a line in the rviz simulator
 * 
 * @param line the line to display
 * @param r the r value in rgb color scheme
 * @param g the g value in rgb color scheme
 * @param b the b value in rgb color scheme
 * @param id the ID to assign to the line
 */
void displayLine(line line, float r, float g, float b, int id) {
    uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();

    marker.ns = "shapes";
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    // Calculate two points from which to build the line
    geometry_msgs::Point pt1;
    pt1.x = -100;
    pt1.y = (getSlope(line) * pt1.x) + getYInt(line);
    pt1.z = 0;

    pt1.x += x;
    pt1.y += y;

    geometry_msgs::Point pt2;
    pt2.x = 100;
    pt2.y = (getSlope(line) * pt2.x) + getYInt(line);
    pt2.z = 0;

    pt2.x += x;
    pt2.y += y;

    marker.points.push_back(pt1);
    marker.points.push_back(pt2);

    // Set the thickness of the line (0-1)
    marker.scale.x = 0.15;

    // Set the color of the line
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.pose.orientation.w = 1;

    marker.lifetime = ros::Duration(1.0);
    markerPub.publish(marker);
}

/**
 * @brief Calculate the slope of a line in standard form
 * 
 * @param line the line
 * @return the slope of the line
 */
double getSlope(line line) {
    return -(line.a / line.b);
}

/**
 * @brief Calculate the Y-intercept of a line in standard form
 * 
 * @param line the line
 * @return the y-intercept of the line
 */
double getYInt(line line) {
    return -(line.c / line.b);
}

/**
 * @brief Determine the distance from a point to a line in standard form
 * 
 * @param x the x of the point
 * @param y the y of the point
 * @param line the line
 * 
 * @return the distance from the point to the line in standard form
 */
double lineToPtDistance(double x, double y, line l) {
    return (std::abs((l.a * x) + (l.b * y) + l.c) / (std::sqrt(l.a*l.a + l.b*l.b)));
}

/**
 * @brief Determine the angle from a point to a line
 * 
 * @param x the x of the point
 * @param y the y of the point
 * @param line the line
 * 
 * @return the angle from the point to the line (parallel means 90)
 */
double lineToPtAngle(double x, double y, line l) {
    return M_PI_2 + std::atan(getSlope(l));
}

/**
 * @brief Determine the standard form of a line given a polar representation
 * 
 * @param distance the r to the line (distance from the origin)
 * @param theta the theta to the line (angle from the origin)
 * 
 * @return a line containing correct a, b, and c values corresponding to the line represented by the method arguments
 */
line polarToStandard(double distance, double theta) {
    line answer;
    answer.distance = distance;
    answer.theta = theta;
    double slope = -1 / (std::tan(theta));
    double yInt = -(distance * std::cos(theta)) * slope + (distance * std::sin(theta));
    answer.a = (100 * slope + yInt) - (-100 * slope + yInt);
    answer.b = -100 - 100;
    answer.c = -((answer.a * -100) + (answer.b * (-100 * slope + yInt)));
    return answer;
}

/**
 * @brief Start or reset the Kalman filter
 */
void startKalmanFilter() {
    // Input matrices
    Eigen::Matrix2d initialCovariance, modelError, measurementError;

    initialCovariance << initialCovarianceInput[0][0], initialCovarianceInput[0][1],
                         initialCovarianceInput[1][0], initialCovarianceInput[1][1];

    modelError << initialModelErrorInput[0][0], initialModelErrorInput[0][1],
                  initialModelErrorInput[1][0], initialModelErrorInput[1][1];

    measurementError << initialMeasurementErrorInput[0][0], initialMeasurementErrorInput[0][1],
                        initialMeasurementErrorInput[1][0], initialMeasurementErrorInput[1][1];

    // Find the initial state so the transition matrix can begin
    Eigen::MatrixXd initialState = Eigen::MatrixXd::Zero(2,1);

    for(int i{0}; i < numInitialStateDetections; i++) {
        PointCloud::ConstPtr pcl {ros::topic::waitForMessage<PointCloud>("/velodyne_points")};
        lineGroup lines {ransac(pcl)};
        initialState(0,0) = initialState(0,0) + lines.lines.at(0).distance;
        initialState(1,0) = initialState(1,0) + lines.lines.at(0).theta;
    }

    initialState(0,0) = initialState(0,0) / numInitialStateDetections;
    initialState(1,0) = initialState(1,0) / numInitialStateDetections;

    filter = Kalman(x, y, yaw, initialState, initialCovariance, modelError, measurementError);
}

/**
 * @brief Update the output to the robot using Lidar data, through a PID controller
 * 
 * @param pcl the message received from the Lidar ros topic ("/velodyne_points")
 */
void velodyneCallBack(const PointCloud::ConstPtr &pcl) {
    ros::Time currentTime = ros::Time::now();
    if(firstLidarCallback) {
        Tglobal_lastFrame << std::cos(-yaw), std::sin(-yaw), x,
                            -std::sin(-yaw), std::cos(-yaw), y,
                            0              , 0             , 1;
        firstLidarCallback = false;
    }
    else if(currentTime.toSec() - lastMotionUpdate.toSec() > 0.1) {
        // Determine the ratio of points ahead of the robot
        int numValidPts {0};
        int numAheadPts {0};
        for(int i{0}; i < pcl->width; i++) {
            if(!isGroundPoint(pcl->points.at(i))) {
                numValidPts++;
                if(pcl->points.at(i).x > 0) numAheadPts++;
            }
        }
        double aheadRatio {static_cast<double>(numAheadPts) / numValidPts};

        // Decide if we should switch modes
        if(!turning) {
            if(aheadRatio < endOfRowRatio) endOfRowCounter++;
            else if (endOfRowCounter > 0) endOfRowCounter--;
            if(endOfRowCounter > endOfRowThreshold) {
                // Execute if robot is driving and should now start turning
                turning = true;
                endOfRowCounter = 0;
                ROS_INFO("START TURNING MODE");
            }
        } else {
            if(aheadRatio > startOfRowRatio) startOfRowCounter++;
            else if (startOfRowCounter > 0) startOfRowCounter--;
            if(startOfRowCounter > startOfRowThreshold) {
                // Execute if robot is turning and should now start driving
                turning = false;
                directionForward = !directionForward;
                startOfRowCounter = 0;
                ros::spinOnce();
                startKalmanFilter();
                ros::Duration(0.3).sleep();
                ROS_INFO("START DRIVING MODE");
            }
        }

        if(turning) {
            // Construct a velocity message to give to the Husky based on constant angular velocity turning
            geometry_msgs::Twist twist;

            twist.linear.x = turningSpeed - 0.1;
            twist.linear.y = 0;
            twist.linear.z = 0;

            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = turningSpeed / (distanceBetweenRows / 2);

            currentVelocityMessage = twist;
            lastMotionUpdate = lastVelocityUpdate = currentTime;
        } else {
            // Collect the detected lines from the RANSAC algorithm
            lineGroup lines {ransac(pcl)};

            Eigen::MatrixXd detectedState(2,1);
            detectedState << lines.lines.at(0).distance, lines.lines.at(0).theta;

            // Determine the transition model parameters for the EKF by changing the frame of reference from global to local
            Eigen::Matrix3d Tglobal_thisFrame;
            Tglobal_thisFrame << std::cos(-yaw), std::sin(-yaw), x,
                                -std::sin(-yaw), std::cos(-yaw), y,
                                0              , 0             , 1;
            
            Eigen::Matrix3d TlastFrame_thisFrame;
            TlastFrame_thisFrame = Tglobal_lastFrame.inverse() * Tglobal_thisFrame;

            double deltaX {TlastFrame_thisFrame(0,2)};
            double deltaY {TlastFrame_thisFrame(1,2)};
            
            // Pass the RANSAC output and odometry into an EKF for tracking (and smoothing)
            Eigen::MatrixXd outputState(2,1);
            outputState = filter.filter(deltaX, deltaY, yaw, detectedState);

            Tglobal_lastFrame = Tglobal_thisFrame;

            // Print data to a csv file for graphing and testing
            kalmanGraphing << counter << "," << lines.lines.at(0).distance << "," << lines.lines.at(0).theta << "," <<
                            outputState(0,0) << "," << outputState(1,0) << "," <<
                            lineToPtDistance(x, y, groundTruthLine) - 0.1 << "," << M_PI_2 - yaw << "\n";
            counter++;

            if(!(lines.lines.size() == 0)) {
                // Display the line from the Kalman Filter
                displayLine(polarToStandard(outputState(0,0), outputState(1,0)), 0, 0, 1, 2);

                // Determine the desired angle of the robot based on distance from the middle
                double desiredAngle {((outputState(0,0) - (distanceBetweenRows / 2)) / (distanceBetweenRows / 2))};

                double tempYaw = yaw;

                // If the robot is driving the other direction, rotate the desired angle by 180 degrees
                if(!directionForward) {
                    tf::Quaternion q_current = tf::createQuaternionFromYaw(yaw);
                    tf::Quaternion q_rotation = tf::createQuaternionFromRPY(0, 0, M_PI);
                    tempYaw = tf::getYaw(q_current * q_rotation);
                }

                // Use a PID to get the robot to the desired angle, given the current angle
                controller.setSetPoint(desiredAngle);
                double angVel {controller.calculate(tempYaw, currentTime)};

                //Construct a velocity message to give to the Husky
                geometry_msgs::Twist twist;

                twist.linear.x = drivingSpeed;
                twist.linear.y = 0;
                twist.linear.z = 0;

                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = angVel;

                currentVelocityMessage = twist;
                lastMotionUpdate = lastVelocityUpdate = currentTime;
            } else {
                ROS_INFO("No Valid Line Group Found by RANSAC! Skipping this loop");
            }
        }
    }
}

/**
 * @brief Perform a RANSAC line detection on an XYZ Point Cloud
 * 
 * @param pcl the point cloud received from the Lidar (vlp16)
 * @return the group of lines that the line detection yielded
 */
lineGroup ransac(const PointCloud::ConstPtr &pcl) {
    lineGroup bestLineGroup;
    bestLineGroup.totalInliers = 0;

    line placeholder;
    placeholder.inliers = 0;
    bestLineGroup.lines.push_back(placeholder);

    for(int i{0}; i < numRansacLoops; i++) {
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
        line currentLine;
        currentLine.a = pcl->points.at(pt2Idx).y - pcl->points.at(pt1Idx).y;
        currentLine.b = pcl->points.at(pt1Idx).x - pcl->points.at(pt2Idx).x;
        currentLine.c = -((currentLine.a * pcl->points.at(pt1Idx).x) + (currentLine.b * pcl->points.at(pt1Idx).y));

        // Determine the number of inliers in the line
        currentLine.inliers = 0;
        for(int j{0}; j < pcl->width; j++) {
            if(!isGroundPoint(pcl->points.at(j))) {
                double distance {lineToPtDistance(pcl->points.at(j).x, pcl->points.at(j).y, currentLine)};
                if(std::abs(distance) <= inlierThreshold) currentLine.inliers++;
            }
        }

        // Determine the current distance and angle to the line
        currentLine.distance = lineToPtDistance(0, 0, currentLine);
        currentLine.theta = lineToPtAngle(0, 0, currentLine);

        if(getYInt(currentLine) > 0) { // We will only proceed if the detected line is to the left, for consistency
            // Construct a line group containing the original line
            lineGroup currentLineGroup;
            currentLineGroup.lines.push_back(currentLine);
            currentLineGroup.totalInliers = currentLine.inliers;

            // TODO:
            // I'm still unsure of whether I'm going to use lineGroups or just single lines to determine the best one. This will still be relevant
            // when implementing the U-Net row detector.

            // Construct a complementary line that is parallel to the original line and add it to the line group (this is the row to the right)
            // line rightLine;
            // rightLine.a = currentLine.a;
            // rightLine.b = currentLine.b;
            // rightLine.c = -(distanceBetweenRows * currentLine.b) + currentLine.c;

            // Determine the number of inliers in the line
            // rightLine.inliers = 0;
            // for(int j{0}; j < pcl->width; j++) {
            //     if(!isGroundPoint(pcl->points.at(j))) {
            //         double distance {lineToPtDistance(pcl->points.at(j).x, pcl->points.at(j).y, rightLine)};
            //         if(std::abs(distance) <= inlierThreshold) rightLine.inliers++;
            //     }
            // }

            // Determine the current distance and angle to the line
            // rightLine.distance = lineToPtDistance(0, 0, rightLine);
            // rightLine.theta = lineToPtAngle(0, 0, rightLine);

            // currentLineGroup.lines.push_back(rightLine);
            //currentLineGroup.totalInliers += rightLine.inliers;

            // Determine if this is the best line group
            if(currentLineGroup.totalInliers >= bestLineGroup.totalInliers) bestLineGroup = currentLineGroup;
        }
    }
    return bestLineGroup;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_controller");

    ROS_INFO("Starting wall_controller node");

    if(argc != 4) {
        std::cout << "Expected 3 arguments: P, I, D" << std::endl;
        return -1;
    }

    ros::NodeHandle n;

    // For storing values to determine turning or driving mode
    endOfRowCounter = 0;
    startOfRowCounter = 0;
    counter = 0;
    firstLidarCallback = true;
    directionForward = true;

    // Use the ground truth wall data to compare filtered results from the Kalman filter with truth data
    groundTruthLine.a = 0;
    groundTruthLine.b = 1;
    groundTruthLine.c = -1.5;

    kalmanGraphing = std::ofstream("kalmanGraphing.csv");

    // Seed the STL random number generation with the current time
    std::srand(std::time(nullptr));

    //Kalman Filter Setup
    // Set the initial robot odometry since ROS has not yet spun, and no odometry has yet been collected
    nav_msgs::Odometry::ConstPtr initialOdom = {ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered")};
    odomCallback(initialOdom);

    startKalmanFilter();

    // PID Controller setup
    controller = PID(std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]));

    controller.setInverted(true);
    controller.setSetPoint(directionForward ? 0 : M_PI);
    controller.setOutputLimits(-maxPIDOutput, maxPIDOutput);
    controller.setMaxIOutput(maxIOutput);

    lastMotionUpdate = ros::Time::now();

    // Setup pubs and subs
    ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 50, odomCallback);
    ros::Subscriber subVelodyne = n.subscribe<PointCloud>("/velodyne_points", 50, velodyneCallBack);
    ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    markerPub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    ROS_INFO("Finished Setup, Starting Spinning");

    geometry_msgs::Twist zeroVelocity;
    zeroVelocity.linear.x = 0;
    zeroVelocity.linear.y = 0;
    zeroVelocity.linear.z = 0;
    zeroVelocity.angular.x = 0;
    zeroVelocity.angular.y = 0;
    zeroVelocity.angular.z = 0;

    currentVelocityMessage = zeroVelocity;
    lastVelocityUpdate = ros::Time::now();

    while(ros::ok()) {
        if((ros::Time::now() - lastVelocityUpdate).toSec() > maxTimeBetweenVeloUpdates) velPub.publish(zeroVelocity);
        else velPub.publish(currentVelocityMessage);
        ros::spinOnce();
        std::cout << directionForward << std::endl;
    }

    ROS_INFO("ROS Stopped. Ending Node");

    // Actions to perform at the end of the program
    kalmanGraphing.close();
    return 0;
}