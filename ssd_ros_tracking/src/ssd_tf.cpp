#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/point_cloud_conversion.h>
#include<nav_msgs/Odometry.h>

#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<boost/thread.hpp>

#include<tf/transform_listener.h>

#include<ssd_ros_msgs/Cluster.h>
#include<ssd_ros_msgs/ClusterArray.h>

using namespace std;

typedef pcl::PointXYZRGB PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

class ssdTF
{
    private:
        ros::Time t_points;
        ros::Time t_centroid;
        tf::TransformListener tflistener_points;
        tf::TransformListener tflistener_centroid;
        ros::Publisher pub;
        ros::Subscriber sub;
    public:
        ssdTF(ros::NodeHandle& n);
        void callback(const ssd_ros_msgs::ClusterArray& msg);
};

ssdTF::ssdTF(ros::NodeHandle& n){
    sub = n.subscribe("/cluster/array/ssd", 1, &ssdTF::callback, this);
    pub = n.advertise<ssd_ros_msgs::ClusterArray>("/cluster/array/tf", 1);
}

void ssdTF::callback(const ssd_ros_msgs::ClusterArray& msg){
    ssd_ros_msgs::ClusterArray clusters;
    ssd_ros_msgs::ClusterArray clusters_global;
    clusters = msg;
    size_t size = clusters.clusters.size();
    cout<<"size:"<<size<<endl;
    // printf("Array size:%d\n", clusters.clusters.size());

    for(size_t i=0;i<size;i++){
        // cout<<"No : "<<i<<endl;
        ssd_ros_msgs::Cluster cluster;
        ssd_ros_msgs::Cluster cluster_global;
        
        cluster = clusters.clusters[i];
        
        // points
        sensor_msgs::PointCloud  pc_points;
        sensor_msgs::PointCloud  pc_points_global;
        sensor_msgs::PointCloud2 pc2_points_global;
 
        // centroid
        sensor_msgs::PointCloud  pc_centroid;
        sensor_msgs::PointCloud  pc_centroid_global;
        sensor_msgs::PointCloud2 pc2_centroid_global;
        
        // stamp
        t_points = cluster.points.header.stamp;
        t_centroid = cluster.centroid.header.stamp;

        // pointscloud2 >> pointcloud
        sensor_msgs::convertPointCloud2ToPointCloud(cluster.points, pc_points);
        sensor_msgs::convertPointCloud2ToPointCloud(cluster.centroid, pc_centroid);

        // frame_id stamp を指定
        pc_points.header.frame_id = "/zed_left_camera";
        pc_points.header.stamp = t_points;
        pc_centroid.header.frame_id = "/zed_left_camera";
        pc_centroid.header.stamp = t_centroid;

        // pc_pointsを matching_base_link >> map座標系へ
        try{
            tflistener_points.waitForTransform("/map", "/zed_left_camera", t_points, ros::Duration(1.0));
            tflistener_points.transformPointCloud("/map", t_points, pc_points, "/zed_left_camera", pc_points_global);
            sensor_msgs::convertPointCloudToPointCloud2(pc_points_global, pc2_points_global);
            pc2_points_global.header.frame_id = "/map";
        }catch(tf::TransformException ex){
            ROS_ERROR("%s\n", ex.what());
        }

        // pc_centroidを matching_base_link >> odom座標系へ
        try{
            tflistener_centroid.waitForTransform("/map", "zed_left_camera", t_centroid, ros::Duration(1.0));
            tflistener_centroid.transformPointCloud("/map", t_centroid, pc_centroid, "/zed_left_camera", pc_centroid_global);
            sensor_msgs::convertPointCloudToPointCloud2(pc_centroid_global, pc2_centroid_global);
            pc2_centroid_global.header.frame_id = "/map";
        }catch(tf::TransformException ex){
            ROS_ERROR("%s\n", ex.what());
        }

        cluster_global.points   = pc2_points_global;
        cluster_global.centroid = pc2_centroid_global;
        clusters_global.clusters.push_back(cluster_global);

        CloudA pcl_points;
        CloudA pcl_centroid;

        fromROSMsg(cluster_global.centroid, pcl_centroid);
        fromROSMsg(cluster_global.points, pcl_points);

        cout<<"No : "<<i<<" centroid : "<< pcl_centroid.points.size()<< " points : " << pcl_points.points.size()<<endl;


    }
    cout<<"final size : "<<clusters.clusters.size()<<endl;
    pub.publish(clusters_global);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ssd_tf");
    ros::NodeHandle n;

    ssdTF run(n);

    ros::spin();

    return 0;
}

