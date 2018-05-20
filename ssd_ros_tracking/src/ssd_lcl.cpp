#include <ros/ros.h>
#include "ros/package.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <ssd_ros_msgs/BoundingBox.h>
#include <ssd_ros_msgs/BoundingBoxArray.h>
#include <ssd_ros_msgs/SSD.h>
#include <ssd_ros_msgs/Cluster.h>
#include <ssd_ros_msgs/ClusterArray.h>

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZRGB PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

ros::Publisher pub;

void clusterCallback(const ssd_ros_msgs::ClusterArray& msg)
{
    ssd_ros_msgs::ClusterArray clusters;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ssd_lcl");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cluster/array/ssd", 10, clusterCallback);


    ros::Rate rate(20);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
