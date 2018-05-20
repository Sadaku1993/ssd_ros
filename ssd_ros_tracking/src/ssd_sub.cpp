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

bool cluster_flag = false;

ros::Publisher pub_centroid;
ros::Publisher pub_points;

ssd_ros_msgs::ClusterArrayConstPtr cluster_;
void clusterCallback(const ssd_ros_msgs::ClusterArrayConstPtr& msg)
{
    printf("subscribe\n");
    cluster_ = msg;
    cluster_flag = true;
    size_t size = cluster_->clusters.size();
    cout<<"size:"<<size<<endl;
    for(size_t i=0;i<size;i++)
    {
        CloudA pcl_centroid;
        CloudA pcl_points;
        // ssd_ros_msgs::Cluster cluster;
        sensor_msgs::PointCloud2 centroid;
        sensor_msgs::PointCloud2 points;
        // cluster = cluster_->clusters[i];
        // centroid = cluster.centroid;
        // points = cluster.points;
        centroid = cluster_->clusters[i].centroid;
        points   = cluster_->clusters[i].points;
        
        fromROSMsg(centroid, pcl_centroid);
        fromROSMsg(points, pcl_points);

        cout<<"No : "<<i<<" centroid : "<< pcl_centroid.points.size()<< " points : " << pcl_points.points.size()<<endl;

        //centroid.header.frame_id ="/zed_left_camera";
        centroid.header.frame_id ="/map";
        centroid.header.stamp = ros::Time::now();
        //points.header.frame_id = "/zed_left_camera";
        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();
        pub_centroid.publish(centroid);
        pub_points.publish(points);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ssd_sub");
    ros::NodeHandle n;
    
    // ros::Subscriber ssd_sub = n.subscribe("/cluster/array/ssd", 10, clusterCallback);
    ros::Subscriber ssd_sub = n.subscribe("/cluster/array/tf", 10, clusterCallback);
    pub_centroid = n.advertise<sensor_msgs::PointCloud2>("/centroid", 1);
    pub_points = n.advertise<sensor_msgs::PointCloud2>("/points", 1);

    ros::Rate rate(20);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

