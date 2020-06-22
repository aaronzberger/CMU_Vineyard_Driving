#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <numeric>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

double yaw;
int currentTime;
ros::Publisher markerPub;
visualization_msgs::Marker marker;
bool firstTime;

bool wallPoint(const pcl::PointXYZ &pt) {
    return (!(pt.z > -0.375 && pt.z < -0.365));
}


void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

    tf::Quaternion quat;
    quat.setW(msg->pose.pose.orientation.w);
    quat.setX(msg->pose.pose.orientation.x);
    quat.setY(msg->pose.pose.orientation.y);
    quat.setZ(msg->pose.pose.orientation.z);

    yaw = tf::getYaw(quat);
}

void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg) {
    currentTime = (msg->clock.sec * 1000) + int(msg->clock.nsec / 1e6);
}

void velodyneCallBack(const PointCloud::ConstPtr &msg) {
    //ROS_INFO("Velo callback");
    
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }

    if(firstTime) {
        std::ofstream csvFile("lineGraphing.csv");
        BOOST_FOREACH(const pcl::PointXYZ &pt, msg->points) {
            csvFile << pt.x << "," << pt.y << "\n";
        }
        csvFile.close();
    }

    if(firstTime) {
        std::ofstream csvFile1("lineGraphingA.csv");
        for(int i{0}; i < msg->width - 0; i++) {
            if(wallPoint(msg->points.at(i))) {
                csvFile1 << msg->points.at(i).x << "," << msg->points.at(i).y << "\n";
            }
        }
    }

    double slope, yInt, sumX, sumY, sumXY, sumX2;
    // for(pcl::PointXYZ pt: msg->points) {
    //     sumX += pt.x;
    //     sumY += pt.y;
    //     sumXY += pt.x * pt.y;
    //     sumX2 += pt.x * pt.x;
    // }
    int numPoints {0};
    for(int i{0}; i < msg->width - 0; i++) {
        if(wallPoint(msg->points.at(i))) {
            sumX += msg->points.at(i).x;
            sumY += msg->points.at(i).y;
            sumXY += msg->points.at(i).x * msg->points.at(i).y;
            sumX2 += msg->points.at(i).x * msg->points.at(i).x;
            numPoints++;
        }
    }
    double xMean = sumX / numPoints;
    double yMean = sumY / numPoints;
    double denominator = sumX2 - sumX * xMean;
    slope = (sumXY - sumX * yMean) / denominator;
    yInt = yMean - slope * xMean;

    // size_t n = msg->width;

    // double numerator = 0.0;
    // double denominator = 0.0;

    // for(size_t i=0; i<n; ++i){
    //     numerator += (msg->points.at(i).x - xMean) * (msg->points.at(i).y - yMean);
    //     denominator += (msg->points.at(i).x - xMean) * (msg->points.at(i).x - xMean);
    // }

    // slope = numerator / denominator;

    std::cout << "Slope: " << slope << ", Yint: " << yInt <<std::endl;
    std::cout << "Num Points: " << numPoints << " out of " << msg->width << std::endl;

    double angle = std::atan(slope);
    std::cout << angle * (180 / 3.1415)  << std::endl;
    
    // ros::Rate r(1);

    //     marker.header.frame_id = "base_link";
    //     marker.header.stamp = ros::Time();

    //     marker.ns = "basic_shapes";
    //     marker.id = 0;

    //     marker.type = visualization_msgs::Marker::ARROW;
    //     marker.action = visualization_msgs::Marker::ADD;

    //     marker.pose.position.x = 1;
    //     marker.pose.position.y = 1;
    //     marker.pose.position.z = 1;
    //     marker.pose.orientation.x = 0.0;
    //     marker.pose.orientation.y = 0.0;
    //     marker.pose.orientation.z = 0.0;
    //     marker.pose.orientation.w = 1.0;

    //     marker.scale.x = 1;
    //     marker.scale.y = 0.1;
    //     marker.scale.z = 0.1;

    //     marker.color.r = 1.0;
    //     marker.color.g = 0.0;
    //     marker.color.b = 0.0;
    //     marker.color.a = 1.0;

    //     // tf::Vector3 axisVector(0,0,0);
    //     // tf::Vector3 upVector(1,0,0);
    //     // tf::Vector3 rightVector = axisVector.cross(upVector);
    //     // rightVector.normalized();
    //     // tf::Quaternion q(rightVector, -1 * acos(axisVector.dot(upVector)));
    //     // q.normalize();
    //     // geometry_msgs::Quaternion lineOrientation;
    //     // tf::quaternionTFToMsg(q, lineOrientation);

    //     //marker.lifetime = ros::Duration();
    //     while(markerPub.getNumSubscribers() < 1) {
    //         ROS_WARN_ONCE("Please create a subscriber to the marker");
    //         //sleep(1);
    //     }
    //     ROS_INFO("Pub marker");
    //     markerPub.publish(marker);
    firstTime = false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_controller");

    ROS_INFO("STARTED MAIN");

    firstTime = true;
    ros::NodeHandle n;

    ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, odomCallback);
    ros::Subscriber subClock = n.subscribe<rosgraph_msgs::Clock>("/clock", 1000, clockCallback);
    ros::Subscriber subVelodyne = n.subscribe<PointCloud>("/velodyne_points", 1000, velodyneCallBack);
    markerPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

    ros::spin();

    return 0;
}