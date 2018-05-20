#include<ros/ros.h>
#include<std_msgs/Bool.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/point_cloud.h>

#include<pcl/point_types.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/features/normal_3d.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/kdtree/kdtree.h>
#include<pcl/segmentation/extract_clusters.h>

#include<Eigen/Core>
#include<boost/thread.hpp>

#include<ssd_ros_msgs/Cluster.h>
#include<ssd_ros_msgs/ClusterArray.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

bool pc_flag = false;   // for callback

ros::Publisher pub;

// pointcloudの重心点を算出
void getClusterInfo(CloudA pt, PointA& cluster)
{
    Vector3f centroid;
	centroid[0]=pt.points[0].x;
	centroid[1]=pt.points[0].y;
	centroid[2]=pt.points[0].z;
    
	Vector3f min_p;
	min_p[0]=pt.points[0].x;
	min_p[1]=pt.points[0].y;
    min_p[2]=pt.points[0].z;
    
	Vector3f max_p;
	max_p[0]=pt.points[0].x;
	max_p[1]=pt.points[0].y;
	max_p[2]=pt.points[0].z;
	
	for(size_t i=1;i<pt.points.size();i++){
        centroid[0]+=pt.points[i].x;
        centroid[1]+=pt.points[i].y;
        centroid[2]+=pt.points[i].z;
		if (pt.points[i].x<min_p[0]) min_p[0]=pt.points[i].x;
		if (pt.points[i].y<min_p[1]) min_p[1]=pt.points[i].y;
        if (pt.points[i].z<min_p[2]) min_p[2]=pt.points[i].z;
		
		if (pt.points[i].x>max_p[0]) max_p[0]=pt.points[i].x;
		if (pt.points[i].y>max_p[1]) max_p[1]=pt.points[i].y;
		if (pt.points[i].z>max_p[2]) max_p[2]=pt.points[i].z;
    }
    
	cluster.x=centroid[0]/(float)pt.points.size();
    cluster.y=centroid[1]/(float)pt.points.size();
    cluster.z=centroid[2]/(float)pt.points.size();
}


void cpu_clustering(CloudAPtr pcl_cloud, ssd_ros_msgs::ClusterArray& clusters)
{
    //Downsample//
    pcl::VoxelGrid<PointA> vg;  
	CloudAPtr ds_cloud (new CloudA);  
	vg.setInputCloud (pcl_cloud);  
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*ds_cloud);
    //downsampled point's z =>0
    vector<float> tmp_z;
    tmp_z.resize(ds_cloud->points.size());
	for(int i=0;i<(int)ds_cloud->points.size();i++){
        tmp_z[i]=ds_cloud->points[i].z;
		ds_cloud->points[i].z  = 0.0;
    }
    //Clustering//
    pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
    tree->setInputCloud (ds_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointA> ec;
    ec.setClusterTolerance (0.05); // 15cm
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (4000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(ds_cloud);
    ec.extract (cluster_indices);
    
    //reset z value
	for(int i=0;i<(int)ds_cloud->points.size();i++)
        ds_cloud->points[i].z=tmp_z[i];
    size_t num=0;
    
    //get cluster information//
    for(size_t iii=0;iii<cluster_indices.size();iii++){
        ssd_ros_msgs::Cluster cluster;

        CloudA cluster_points;
        CloudA cluster_centroid;

        sensor_msgs::PointCloud2 pc2_points;
        sensor_msgs::PointCloud2 pc2_centroid;

        cluster_points.points.resize(cluster_indices[iii].indices.size()+num);
        cluster_centroid.points.resize(cluster_indices[iii].indices.size());

        for(size_t jjj=0;jjj<cluster_indices[iii].indices.size();jjj++){
            int p_num = cluster_indices[iii].indices[jjj];
            cluster_points.points[jjj] = pcl_cloud->points[p_num];
        }
        getClusterInfo(cluster_points, cluster_centroid[iii]);

        toROSMsg(cluster_points, pc2_points);
        pc2_points.header.frame_id = "/velodyne";
        pc2_points.header.stamp = ros::Time::now();

        toROSMsg(cluster_centroid, pc2_centroid);
        pc2_centroid.header.frame_id = "/velodyne";
        pc2_centroid.header.stamp = ros::Time::now();

        cluster.points   = pc2_points;
        cluster.centroid = pc2_centroid;
        clusters.clusters.push_back(cluster);
    }
}


void cluster(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
    CloudAPtr pcl_cloud (new CloudA);
    CloudA pcl_cluster_centroid;
    CloudA pcl_cluster_points;

    ssd_ros_msgs::ClusterArray clusters;

    int cluster_count = 0;

    pcl::fromROSMsg(*pc_msg, *pcl_cloud);

    cpu_clustering(pcl_cloud, clusters);

    pub.publish(clusters);
}

sensor_msgs::PointCloud2ConstPtr pc_;
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pc_ = msg;
    pc_flag = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/velodye_points",10,pcCallback);
    pub = n.advertise<ssd_ros_msgs::ClusterArray>("/cluster/array", 1);

    ros::Rate rate(20);

    while(ros::ok())
    {
        if(pc_flag) cluster(pc_);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

