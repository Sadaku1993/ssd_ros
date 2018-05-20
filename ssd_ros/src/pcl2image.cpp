#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>

bool pc_flag = false;

ros::Publisher pub_image;

sensor_msgs::PointCloud2ConstPtr pc_;
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pc_ = msg;
    pc_flag = true;
}

void pcl2image(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    printf("convert\n");
    sensor_msgs::Image image;
    sensor_msgs::PointCloud2ConstPtr cloud = msg;
    if ((cloud->width * cloud->height) == 0)
        return; //return if the cloud is not dense!
    try
    {
        pcl::toROSMsg (*cloud, image); //convert the cloud
    }
    catch (std::runtime_error e)
    {
        ROS_ERROR_STREAM("Error in converting cloud to image message: "
                << e.what());
    }
    pub_image.publish(image);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl2image");
    ros::NodeHandle n;

    ros::Subscriber pc_sub = n.subscribe("cloud", 10, pcCallback);
    pub_image = n.advertise<sensor_msgs::Image>("pcl_image", 10);

    ros::Rate rate(20);

    while(ros::ok())
    {
        if(pc_flag) pcl2image(pc_);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

