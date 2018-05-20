#include<ssd_ros/remove_plane.h>

using namespace std;
using namespace Eigen;

ros::Publisher pub_detect;
ros::Publisher pub_remove;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*msg, *cloud);
    int num_pt = cloud->points.size();

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_detect (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_remove (new pcl::PointCloud<pcl::PointNormal>);
    
    for(int i=0;i<num_pt;i++)
    {
        if (0.005<cloud->points[i].curvature){
            cloud_detect->points.push_back(cloud->points[i]);
        }
        else{
            cloud_remove->points.push_back(cloud->points[i]);
        }
    }

    printf("Input size:%d Output size:%d Remove size:%d\n", 
            int(cloud->points.size()), int(cloud_detect->points.size()), int(cloud_remove->points.size()));

    sensor_msgs::PointCloud2 pc2_detect;
    pcl::toROSMsg(*cloud_detect, pc2_detect);
    pc2_detect.header.frame_id = "velodyne";
    pc2_detect.header.stamp    = ros::Time::now();
    pub_detect.publish(pc2_detect);

    sensor_msgs::PointCloud2 pc2_remove;
    pcl::toROSMsg(*cloud_remove, pc2_remove);
    pc2_remove.header.frame_id = "velodyne";
    pc2_remove.header.stamp    = ros::Time::now();
    pub_remove.publish(pc2_remove);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_plane");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/velodyne/normal_estimation", 1, pcCallback);
    pub_detect = n.advertise<sensor_msgs::PointCloud2>("/velodyne/rm_normal", 10);
    pub_remove = n.advertise<sensor_msgs::PointCloud2>("/velodyne/rm_normal/remove", 10);
    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
