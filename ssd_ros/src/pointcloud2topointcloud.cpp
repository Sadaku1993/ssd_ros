#include <ros/ros.h>
#include "ros/package.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
// #include<image_transport/image_transport.h>

using namespace std;

ros::Publisher pub;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc);
    pc.header.frame_id = "/velodyne";
    pc.header.stamp = ros::Time::now();
    pub.publish(pc);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "pointcloud2topointcloud");
    ros::NodeHandle n;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber pc_sub    = n.subscribe("/velodyne_points",10,pcCallback);

    pub  = n.advertise<sensor_msgs::PointCloud> ("/velodyne_pointcloud",10);

    ros::Rate rate(20);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
