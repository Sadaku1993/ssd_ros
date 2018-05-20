#include<ssd_ros/normal_estimation.h>

using namespace std;
using namespace Eigen;

ros::Publisher pub;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    //Downsample
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.filter (*ds_cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (ds_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.2);
    ne.compute(*cloud_normals);

    int num_pt = ds_cloud->points.size();
    pcl::PointCloud<pcl::PointNormal>::Ptr normal (new pcl::PointCloud<pcl::PointNormal>);
    normal->points.resize(num_pt);
    cout<<"cloud size"<<ds_cloud->points.size()<<endl;
    cout<<"normal size"<<cloud_normals->points.size()<<endl;
    for(int i=0;i<num_pt; i++){
        normal->points[i].x         = ds_cloud->points[i].x;
        normal->points[i].y         = ds_cloud->points[i].y;
        normal->points[i].z         = ds_cloud->points[i].z;
        normal->points[i].normal_x  = cloud_normals->points[i].normal_x;
        normal->points[i].normal_y  = cloud_normals->points[i].normal_y;
        normal->points[i].normal_z  = cloud_normals->points[i].normal_z;
        normal->points[i].curvature = cloud_normals->points[i].curvature;
    }
    normal->width = 1;
    normal->height = normal->points.size();

    sensor_msgs::PointCloud2 pc2_output;
    pcl::toROSMsg(*normal, pc2_output);
    pc2_output.header.frame_id = "velodyne";
    pc2_output.header.stamp    = ros::Time::now();
    pub.publish(pc2_output);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimation");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/rm_ground", 10, pcCallback);
    pub                 = n.advertise<sensor_msgs::PointCloud2>("/velodyne/normal_estimation", 1);
   
    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

   return 0;
}
