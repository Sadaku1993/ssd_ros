#include <ros/ros.h>
#include "ros/package.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
#include "visualization_tools/bounding_box_array.h"
#include "visualization_tools/bounding_box.h"


#include <ssd_ros_msgs/BoundingBox.h>
#include <ssd_ros_msgs/BoundingBoxArray.h>
#include <ssd_ros_msgs/SSD.h>
#include <ssd_ros_msgs/Cluster.h>
#include <ssd_ros_msgs/ClusterArray.h>

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZRGB PointA;
//typedef pcl::PointXYZINormal PointA;  // 法線情報・反射強度を利用する場合はNormal型に変更
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

ros::Publisher pub_cluster_centroid;
ros::Publisher pub_cluster_points;
ros::Publisher pub_cluster_centroid_max;
ros::Publisher pub_cluster_points_max;
ros::Publisher pub_cluster_array;

string target_frame = "/zed_left_camera";
string source_frame = "/velodyne";

bool pc_flag = false;

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
	// cluster.normal_x=max_p[0]-min_p[0];
	// cluster.normal_y=max_p[1]-min_p[1];
	// cluster.normal_z=max_p[2];
}

bool height_detection(float height)
{
    bool flag = false;
    if(0.5<=height && height<=1.7) flag = true;
    return flag;
}

bool width_detection(float width)
{
    bool flag = false;
    if(0.3<= width && width<=0.8) flag = true;
    return flag;
}

bool depth_detection(float depth)
{
    bool flag = false;
    if (0.2 < depth) flag = true;
    return flag;
}

bool size_detection(float size)
{
    bool flag = false;
    if (200<size) flag = true;
    return flag;
}

bool curvature_detection(float curvature)
{
    bool flag = false;
    if (0.10 < curvature) flag = true;
    return flag;
}

bool object_detection(BoundingBox boundingbox)
{
    cout<<"detection"<<endl;
    bool detect = false;

    float height    = boundingbox.height();
    float width     = boundingbox.width();
    float depth     = boundingbox.depth();
    float curvature = boundingbox.curvature();

    bool height_flag    = height_detection(height);
    bool width_flag     = width_detection(width);
    bool depth_flag     = depth_detection(depth);
    bool curvature_flag = curvature_detection(curvature);

    if(curvature_flag && depth_flag && height_flag && width_flag){
        printf("H:%.2f W:%.2f D:%.2f Curv:%.2f\n", height, width, depth, curvature);
        detect = true;
    }

    return detect;
}

void sampling_cpu_clustering(CloudAPtr pcl_in, CloudA& cluster_centroid, CloudA& cluster_pt, CloudA& max_centroid, CloudA& max_pt)
{
    //Downsample//
    pcl::VoxelGrid<PointA> vg;  
	CloudAPtr ds_cloud (new CloudA);  
	vg.setInputCloud (pcl_in);  
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
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (4000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(ds_cloud);
    ec.extract (cluster_indices);
    //reset z value
	for(int i=0;i<(int)ds_cloud->points.size();i++)
        ds_cloud->points[i].z=tmp_z[i];
    //get cluster information//
    size_t num=0;
    cluster_centroid.resize(cluster_indices.size());

    // visualization_toolを利用して検出したclusterが検出したい対象物であるかを判断
    bool detect_cluster=false;

    for(size_t iii=0;iii<cluster_indices.size();iii++){
        //cluster cenrtroid
        CloudAPtr cloud_cluster (new CloudA);
        cloud_cluster->points.resize(cluster_indices[iii].indices.size());
        //cluster points
        cluster_pt.points.resize(cluster_indices[iii].indices.size()+num);
        for(size_t jjj=0;jjj<cluster_indices[iii].indices.size();jjj++){
            int p_num=cluster_indices[iii].indices[jjj];
            cloud_cluster->points[jjj]=ds_cloud->points[p_num];
            cluster_pt.points[num+jjj]=ds_cloud->points[p_num];
        }
        getClusterInfo(*cloud_cluster, cluster_centroid[iii]);
 
        if (detect_cluster!=true){
            // BoundingBoxArray boundingbox_array;
            // boundingbox_array.push_back<CloudAPtr&>(cloud_cluster);
            // cout << "\033[2J";
            // printf("\033[%d;%dH", 1, 1);
            // cout << boundingbox_array << endl;
            // cout << boundingbox_array.v_bb.curv << endl;
            bool check_cluster = false;
            BoundingBox boundingbox;
            boundingbox.pc2bb(cloud_cluster);
            check_cluster = object_detection(boundingbox);

            if (check_cluster){
                max_pt.points.resize(cloud_cluster->points.size());
                for(size_t jjj=0;jjj<max_pt.points.size();jjj++){
                    max_pt.points[jjj] = cloud_cluster->points[jjj];
                }
                max_centroid.resize(1);
                getClusterInfo(max_pt, max_centroid[0]);
                break;
            }
        }
                   
        // if(iii==0){
        //     max_pt.points.resize(cluster_indices[iii].indices.size()+num);
        //     for(size_t jjj=0;jjj<cluster_indices[iii].indices.size();jjj++){
        //         int p_num=cluster_indices[iii].indices[jjj];
        //         max_pt.points[num+jjj]=ds_cloud->points[p_num];
        //     }
        //     max_centroid.resize(1);
        //     getClusterInfo(*cloud_cluster, max_centroid[iii]);
        // }
        num=cluster_pt.points.size();//save previous size
    }
}   


void cluster(const ssd_ros_msgs::SSDConstPtr& pc_msg)
{
    CloudAPtr pcl_cluster_points(new CloudA);
    CloudAPtr pcl_max_points(new CloudA);
    CloudA pcl_cluster_centroid;
    CloudA pcl_max_centroid;

    ssd_ros_msgs::ClusterArray ssd_clusters;

    BoundingBoxArray boundingbox_array;

    int grobal_count = 0;

    for(size_t i=0;i<pc_msg->ssd_array.size();i++)
    {
        int local_count = 0;
        const sensor_msgs::PointCloud2 &pc2_people = pc_msg->ssd_array[i];
        // printf("Num:%d size:%d\n", i+1, pc2_people.width);
        
        CloudAPtr pcl_people (new CloudA);
        CloudA cluster_points;
        CloudA cluster_centroid;
        CloudA max_centroid;
        CloudA max_points;
        
      
        pcl::fromROSMsg(pc2_people, *pcl_people);
        
		sampling_cpu_clustering(pcl_people, cluster_centroid, cluster_points, max_centroid, max_points);

		for(size_t j=0;j<cluster_centroid.points.size();j++){
            PointA p = cluster_centroid.points[j];
            pcl_cluster_centroid.push_back(p);
            local_count++;
            grobal_count++;
        }
        for(size_t k=0;k<cluster_points.points.size();k++){
            PointA q = cluster_points.points[k];
            pcl_cluster_points->push_back(q);
        }
		// if(0<pcl_cluster_points->points.size()){
		// 	boundingbox_array.push_back<CloudAPtr&>(pcl_cluster_points);
		// }

        // boundingbox_array.push_back<CloudAPtr&>(pcl_cluster_points);

        for(size_t l=0;l<max_centroid.points.size();l++){
            PointA p = max_centroid.points[l];
            pcl_max_centroid.push_back(p);
        }
        for(size_t m=0;m<max_points.points.size();m++){
            PointA q = max_points.points[m];
            pcl_max_points->push_back(q);
        }

        boundingbox_array.push_back<CloudAPtr&>(pcl_max_points);

        if(0<max_points.points.size()){
            ssd_ros_msgs::Cluster ssd_cluster;
            CloudA ssd_centroid;
            CloudA ssd_points;
           
            for(size_t l=0;l<max_centroid.points.size();l++){
                PointA p = max_centroid.points[l];
                ssd_centroid.push_back(p);
            }
            for(size_t m=0;m<max_points.points.size();m++){
                PointA q = max_points.points[m];
                ssd_points.push_back(q);
            }
            toROSMsg(ssd_centroid, ssd_cluster.centroid);
            toROSMsg(ssd_points, ssd_cluster.points);
            ssd_cluster.centroid.header.frame_id = target_frame;
            ssd_cluster.centroid.header.stamp = ros::Time::now();
            ssd_cluster.points.header.frame_id = target_frame;
            ssd_cluster.points.header.stamp = ros::Time::now();
            ssd_clusters.clusters.push_back(ssd_cluster);
        }
        
        // if(0<max_points.points.size()){
        //     toROSMsg(pcl_max_centroid, ssd_cluster.centroid);
        //     toROSMsg(pcl_max_points, ssd_cluster.points);
        //     ssd_cluster.centroid.header.frame_id = target_frame;
        //     ssd_cluster.centroid.header.stamp = ros::Time::now();
        //     ssd_cluster.points.header.frame_id = target_frame;
        //     ssd_cluster.points.header.stamp = ros::Time::now();
        //     ssd_clusters.clusters.push_back(ssd_cluster);
        // }

        // printf("Array Num:%d Size:%d Class:%d\n",(int)i+1, (int)pc2_people.width, local_count);
    }

    printf("Total Class:%d\n", grobal_count);
    printf("Array size:%d\n", (int)ssd_clusters.clusters.size());

    sensor_msgs::PointCloud2 cluster_centroid_ros;
    toROSMsg(pcl_cluster_centroid, cluster_centroid_ros);
    cluster_centroid_ros.header.frame_id=target_frame;
    cluster_centroid_ros.header.stamp=ros::Time::now();
    pub_cluster_centroid.publish(cluster_centroid_ros);

    sensor_msgs::PointCloud2 cluster_points_ros;
    toROSMsg(*pcl_cluster_points, cluster_points_ros);
    cluster_points_ros.header.frame_id=target_frame;
    cluster_points_ros.header.stamp=ros::Time::now();
    pub_cluster_points.publish(cluster_points_ros);

    // cout << "\033[2J";
    // printf("\033[%d;%dH", 1, 1);
    // cout << boundingbox_array << endl;
    boundingbox_array.publish();

    sensor_msgs::PointCloud2 max_centroid_ros;
    toROSMsg(pcl_max_centroid, max_centroid_ros);
    max_centroid_ros.header.frame_id = target_frame;
    pub_cluster_centroid_max.publish(max_centroid_ros);

    sensor_msgs::PointCloud2 max_points_ros;
    toROSMsg(*pcl_max_points,max_points_ros);
    max_points_ros.header.frame_id = target_frame;
    pub_cluster_points_max.publish(max_points_ros);

    pub_cluster_array.publish(ssd_clusters);

}


ssd_ros_msgs::SSDConstPtr pc_;
void pcCallback(const ssd_ros_msgs::SSDConstPtr& msg)
{
    // printf("Human Count : %d\n", msg->ssd_array.size());
    pc_ = msg;
    pc_flag = true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ssd_cluster");
    ros::NodeHandle n;

    ros::Subscriber pc_sub = n.subscribe("/SSD/people",10,pcCallback);
    
    pub_cluster_centroid=n.advertise<sensor_msgs::PointCloud2>("/cluster/centroid",1);
    pub_cluster_points=n.advertise<sensor_msgs::PointCloud2>("/cluster/points",1);
    pub_cluster_centroid_max=n.advertise<sensor_msgs::PointCloud2>("/cluster/centroid/max",1);
    pub_cluster_points_max=n.advertise<sensor_msgs::PointCloud2>("/cluster/points/max",1);
    pub_cluster_array = n.advertise<ssd_ros_msgs::ClusterArray>("/cluster/array/ssd", 1);

    ros::Rate rate(20);

    while(ros::ok())
    {
        if(pc_flag) cluster(pc_);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
