#include<ssd_ros/normal_estimation_clustering.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointNormal PointA;
typedef pcl::PointNormal PointAPtr;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

ros::Publisher pub;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr cloud_input (new CloudA);
    CloudAPtr cloud_output (new CloudA);
    
    pcl::fromROSMsg(*msg, *cloud_input);
    printf("Data size is : %d\n", int(cloud_input->points.size()));

    for(int i=0;i<int(cloud_input->points.size());i++){
        if(0.15 < cloud_input->points[i].curvature) cloud_output->points.push_back(cloud_input->points[i]);
    }

    sensor_msgs::PointCloud2 pc2_output;
    pcl::toROSMsg(*cloud_output, pc2_output);
    pc2_output.header.frame_id = "velodyne";
    pc2_output.header.stamp    = ros::Time::now();
    pub.publish(pc2_output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_clustering");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/perfect_velodyne/normal", 10, pcCallback);
    pub                 = n.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/threshold", 1);
    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

   return 0;
}

