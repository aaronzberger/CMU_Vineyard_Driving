#include <numeric>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>
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
#include <png++/png.hpp>
#include <Eigen/Dense>

// USER PARAMETERS
constexpr double distanceBetweenRows = 3.0;
constexpr double inlierThreshold = 0.39;
constexpr unsigned numRansacLoops = 250;
constexpr double leftRowYInt = 1.5;

constexpr double turningSpeed = 0.4;
constexpr double drivingSpeed = 0.2;

constexpr unsigned numInitialStateDetections = 15;

constexpr double maxPIDOutput = 0.4;
constexpr double maxIOutput = 0.2;

constexpr double initialCovarianceInput[2][2] = {{0.3, 0  },
                                                 {0  , 0.3}};
constexpr double initialModelErrorInput[2][2] = {{0, 0},
                                                 {0, 0}};
constexpr double initialMeasurementErrorInput[2][2] = {{0.25, 0   },
                                                       {0   , 0.25}};


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct line {
    double a, b, c, distance, theta;
    unsigned inliers;
};

struct lineGroup {
    std::vector<line> lines;
    int totalInliers;
};

//For the world two_walls_3m.world, true line is below:
line groundTruthLine;

// Function Prototypes
void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg);
bool isGroundPoint(const pcl::PointXYZ &pt);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void velodyneCallBack(const PointCloud::ConstPtr &pcl);
void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
void displayLine(line line, float r, float g, float b, int id);
double getYaw(double w, double x, double y, double z);
lineGroup ransac(const PointCloud::ConstPtr &pcl);
bool close(line line1, line line2);
double lineToPtDistance(double x, double y, line l);
double lineToPtAngle(double x, double y, line l);
double getSlope(line line);
double getYInt(line line);

double currentTime, yaw, x, y, lastMotionUpdate;
double trueX, trueY, trueYaw;
ros::Publisher velPub, markerPub;
PID controller;
Kalman filter;
int endOfRowCounter;
int startOfRowCounter;
bool turning;
std::ofstream kalmanGraphing;
int counter;
bool firstLoop;
Eigen::Matrix3d Tglobal_lastFrame;

//Storing the past number of valid points for a moving average
std::vector<int> numAheadPtsVec(49, -1);
std::vector<double> lineTrackerFilter(5, -99);

/**
 * @brief Updates the current time variable
 * 
 * @param msg the message received from the clock ros topic
 */
void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg) {
    currentTime = (msg->clock.sec * 1000) + int(msg->clock.nsec / 1e6);
}

/**
 * @brief Whether the point is a point not belonging to a wall
 * 
 * @param pt the point in the point cloud, containing an X, Y, and Z
 * @return a boolean representing whether the point is not on a wall
 */
bool isGroundPoint(const pcl::PointXYZ &pt) {
    return (pt.z > -0.375 && pt.z < -0.365);
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
 * @brief Update the trueX, trueY, and trueYaw global variables
 * 
 * @param msg the message received from the Gazebo model states ros topic
 */
void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
    unsigned idx {0};
    for(int i{0}; i < msg->name.size(); i++) {
        if(msg->name.at(i) == "/") idx = i;
    }
    trueX = {msg->pose.at(idx).position.x};
    trueY = {msg->pose.at(idx).position.y};
    
    trueYaw = getYaw(msg->pose.at(idx).orientation.w, msg->pose.at(idx).orientation.x, msg->pose.at(idx).orientation.y, msg->pose.at(idx).orientation.z);
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

    marker.lifetime = ros::Duration();
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
 * @brief Determine if two lines are too close to be different lines
 * 
 * @param line1 the first line
 * @param line2 the second line
 * 
 * @return whether the two lines represent the same detected line
 */
bool close(line line1, line line2) {
    if(std::abs(-(line1.a/line1.b) - (-(line2.a/line2.b))) < 0.2) {
        return (std::abs((line1.c/line1.b) - (line2.c/line2.b)) < 1.0);
    }
    return false;
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
 * @brief Update the output to the robot using Lidar data, through a PID controller
 * 
 * @param pcl the message received from the Lidar ros topic (velodyne)
 */
void velodyneCallBack(const PointCloud::ConstPtr &pcl) {
    if(firstLoop) {
        Tglobal_lastFrame << std::cos(-trueYaw), std::sin(-trueYaw), trueX,
                            -std::sin(-trueYaw), std::cos(-trueYaw), trueY,
                            0,                   0,                  1;
        firstLoop = false;
    }
    else if(currentTime - lastMotionUpdate > 10) {
        // Collect the detected lines from the RANSAC algorithm
        lineGroup lines {ransac(pcl)};

        Eigen::MatrixXd detectedState(2,1);
        detectedState << lines.lines.at(0).distance, lines.lines.at(0).theta;

        // Determine the transition model parameters for the EKF by changing the frame of reference from global to local
        Eigen::Matrix3d Tglobal_thisFrame;
        Tglobal_thisFrame << std::cos(-trueYaw), std::sin(-trueYaw), trueX,
                            -std::sin(-trueYaw), std::cos(-trueYaw), trueY,
                            0,                   0,                  1;
        
        Eigen::Matrix3d TlastFrame_thisFrame;
        TlastFrame_thisFrame = Tglobal_lastFrame.inverse() * Tglobal_thisFrame;

        double deltaX {TlastFrame_thisFrame(0,2)};
        double deltaY {TlastFrame_thisFrame(1,2)};
        
        // Pass the RANSAC output and odometry into an EKF for tracking (and smoothing)
        Eigen::MatrixXd outputState(2,1);
        outputState = filter.filter(deltaX, deltaY, trueYaw, detectedState);

        // Print data to a csv file for graphing and testing
        kalmanGraphing << counter << "," << lines.lines.at(0).distance << "," << lines.lines.at(0).theta << "," <<
                          outputState(0,0) << "," << outputState(1,0) << "," <<
                          lineToPtDistance(trueX, trueY, groundTruthLine) - 0.1 << "," << M_PI_2 - trueYaw << "\n";
        counter++;

        Tglobal_lastFrame = Tglobal_thisFrame;

        // Determine whethher to turn or drive
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
            if(aheadRatio < 0.2) endOfRowCounter++;
            if(endOfRowCounter > 20) {
                // Execute if robot is driving and should now start turning
                turning = true;
                endOfRowCounter = 0;
            }
        } else {
            if(aheadRatio > 0.75) startOfRowCounter++;
            if(startOfRowCounter > 20) {
                // Execute if robot is turning and should now start driving
                turning = false;
                startOfRowCounter = 0;
                ros::Duration(1.0).sleep();
            }
        }

        if(turning) {
            // Construct a velocity message to give to the Husky based on basic constant angular velocity turning
            geometry_msgs::Twist twist;

            twist.linear.x = turningSpeed - 0.1;
            twist.linear.y = 0;
            twist.linear.z = 0;

            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = turningSpeed / (distanceBetweenRows / 2);

            velPub.publish(twist);
            lastMotionUpdate = currentTime;

        } else if(!(lines.lines.size() == 0)) {
            // Display the originally detected line on the left in blue
            displayLine(lines.lines.at(0), 0, 0, 1, 2);

            //Run the PID controller
            std::cout << "Distance to wall: " << outputState(0,0) << std::endl;

            double angVel {controller.calculate(outputState(0,0), currentTime)};
            
            std::cout << "PID Output: " << angVel << std::endl;
            std::cout << "---------------------------" << std::endl;

            //Construct a velocity message to give to the Husky
            geometry_msgs::Twist twist;

            twist.linear.x = drivingSpeed;
            twist.linear.y = 0;
            twist.linear.z = 0;

            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = angVel;

            velPub.publish(twist);
            lastMotionUpdate = currentTime;
        } else {
            std::cout << "Size " << lines.lines.size() << " Inliers " << lines.totalInliers << std::endl;
            std::cout << "NO LINE GROUP FOUND" << std::endl;
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

            // Construct a complementary line that is parallel to the original line and add it to the line group (this is the row to the right)
            line rightLine;
            rightLine.a = currentLine.a;
            rightLine.b = currentLine.b;
            rightLine.c = -(distanceBetweenRows * currentLine.b) + currentLine.c;

            // Determine the number of inliers in the line
            rightLine.inliers = 0;
            for(int j{0}; j < pcl->width; j++) {
                if(!isGroundPoint(pcl->points.at(j))) {
                    double distance {lineToPtDistance(pcl->points.at(j).x, pcl->points.at(j).y, rightLine)};
                    if(std::abs(distance) <= inlierThreshold) rightLine.inliers++;
                }
            }

            // Determine the current distance and angle to the line
            rightLine.distance = lineToPtDistance(0, 0, rightLine);
            rightLine.theta = lineToPtAngle(0, 0, rightLine);

            currentLineGroup.lines.push_back(rightLine);
            //currentLineGroup.totalInliers += rightLine.inliers;

            // Determine if this is the best line group
            if(currentLineGroup.totalInliers >= bestLineGroup.totalInliers) bestLineGroup = currentLineGroup;
        }
    }
    return bestLineGroup;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_controller");

    if(argc != 4) {
        std::cout << "Expected 3 arguments: P, I, D" << std::endl;
        return -1;
    }

    ros::NodeHandle n;

    // Initialize ground truth for simulations
    groundTruthLine.a = 0;
    groundTruthLine.b = 1;
    groundTruthLine.c = -leftRowYInt;

    // For storing values to determine turning or driving mode
    endOfRowCounter = 0;
    startOfRowCounter = 0;

    // For printing a csv file for graphing
    counter = 0;
    kalmanGraphing = std::ofstream("kalmanGraphing.csv");

    firstLoop = true;

    // Seed the STL random number generation with the current time
    std::srand(std::time(nullptr));

    //Kalman Filter Setup
    Eigen::Matrix2d initialCovariance(initialCovarianceInput), modelError(initialModelErrorInput), measurementError(initialMeasurementErrorInput);

    // Find the initial robot odometry so the EKF can be initialized properly
    gazebo_msgs::ModelStates::ConstPtr initialOdom {ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states")};
    modelStateCallback(initialOdom);

    unsigned numInitialStateDetections {numInitialStateDetections};
    Eigen::MatrixXd initialState = Eigen::MatrixXd::Zero(2,1);

    for(int i{0}; i < numInitialStateDetections; i++) {
        PointCloud::ConstPtr detection {ros::topic::waitForMessage<PointCloud>("/velodyne_points")};
        lineGroup lines {ransac(detection)};
        initialState(0,0) = initialState(0,0) + lines.lines.at(0).distance;
        initialState(1,0) = initialState(1,0) + lines.lines.at(0).theta;
    }

    initialState(0,0) = initialState(0,0) / numInitialStateDetections;
    initialState(1,0) = initialState(1,0) / numInitialStateDetections;

    filter = Kalman(0, 0, trueYaw, initialState, initialCovariance, modelError, measurementError);

    // PID Controller setup
    controller = PID(std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]));

    controller.setInverted(false);
    controller.setSetPoint(distanceBetweenRows / 2);
    controller.setOutputLimits(-maxPIDOutput, maxPIDOutput);
    controller.setMaxIOutput(maxIOutput);

    ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 50, odomCallback);
    ros::Subscriber subModelStates = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 50, modelStateCallback);
    ros::Subscriber subClock = n.subscribe<rosgraph_msgs::Clock>("/clock", 50, clockCallback);
    ros::Subscriber subVelodyne = n.subscribe<PointCloud>("/velodyne_points", 50, velodyneCallBack);
    velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    markerPub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    ros::spin();

    // Actions to perform at the end of the program
    kalmanGraphing.close();

    return 0;
}