#include <ros/ros.h>
#include "ros/package.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <ssd_ros_msgs/Cluster.h>
#include <ssd_ros_msgs/ClusterArray.h>

typedef pcl::PointNormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;
using namespace Eigen;

struct Cluster{
    float x;
    float y;
    float z;

    float width;
    float height;
    float depth;

    float curvature;

};

void getClusterInfo(CloudA pt, Cluster& cluster)
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

    float curvature=pt.points[0].curvature;
	
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

        curvature+=pt.points[i].curvature;
    }
    
	cluster.x=centroid[0]/(float)pt.points.size();
    cluster.y=centroid[1]/(float)pt.points.size();
    cluster.z=centroid[2]/(float)pt.points.size();
    cluster.depth  = max_p[0]-min_p[0];
    cluster.width  = max_p[1]-min_p[1];
    cluster.height = max_p[2]-min_p[2]; 
    cluster.curvature = curvature/(float)pt.points.size();
}

void detection(Cluster cluster, CloudA cloud, ssd_ros_msgs::ClusterArray &cluster_array)
{
    if( 0.2<cluster.width && cluster.width<1.2 &&
        0.5<cluster.height && cluster.height<2.0 &&
        cluster.depth<1.2 ){
        
        ssd_ros_msgs::Cluster data;
        
        data.header.frame_id = "/velodyne";
        data.header.stamp = ros::Time::now();

        data.pose.position.x = cluster.x;
        data.pose.position.y = cluster.y;
        data.pose.position.z = cluster.z;
        data.pose.orientation.x = 0;
        data.pose.orientation.y = 0;
        data.pose.orientation.z = 0;
        data.pose.orientation.w = 1;

        data.width     = cluster.width;
        data.height    = cluster.height;
        data.depth     = cluster.depth;
        data.curvature = cluster.curvature;

        sensor_msgs::PointCloud2 pc2_cloud;
        toROSMsg(cloud, pc2_cloud);
        pc2_cloud.header.frame_id = "/velodyne";
        pc2_cloud.header.stamp = ros::Time::now();

        data.points = pc2_cloud;

        cluster_array.clusters.push_back(data);
    }
}

void clustering(CloudAPtr cloud_in, ssd_ros_msgs::ClusterArray& cluster_array)
{
    //downsampled point's z =>0
    vector<float> tmp_z;
    tmp_z.resize(cloud_in->points.size());
	for(int i=0;i<(int)cloud_in->points.size();i++){
        tmp_z[i]=cloud_in->points[i].z;
		cloud_in->points[i].z  = 0.0;
    }
    //Clustering//
    pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
    tree->setInputCloud (cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointA> ec;
    ec.setClusterTolerance (0.05); // 15cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (2000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud_in);
    ec.extract (cluster_indices);
    //reset z value
	for(int i=0;i<(int)cloud_in->points.size();i++)
        cloud_in->points[i].z=tmp_z[i];

    // int num = int(cluster_indices.size());
    // printf("indices size:%d\n", num);


    for(int iii=0;iii<(int)cluster_indices.size();iii++)
    {
        // cluster points
        CloudAPtr cloud_cluster (new CloudA);
        cloud_cluster->points.resize(cluster_indices[iii].indices.size());
        // cluster data
        Cluster data;
        for(int jjj=0;jjj<int(cluster_indices[iii].indices.size());jjj++){
            int p_num = cluster_indices[iii].indices[jjj];
            cloud_cluster->points[jjj] = cloud_in->points[p_num];
        }
        getClusterInfo(*cloud_cluster, data);
        detection(data, *cloud_cluster, cluster_array);
    }

    // printf("detect size:%d\n", int(cluster_array.clusters.size()));
}

