#include <ssd_clustering/clustering.h>

ros::Publisher pub_bbox;
ros::Publisher pub_cluster;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr cloud (new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    ssd_ros_msgs::ClusterArray cluster_array;
    clustering(cloud, cluster_array);

    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    bbox_array.header.frame_id = "/velodyne";
    bbox_array.header.stamp = ros::Time::now();

    int cluster_size = int(cluster_array.clusters.size());

    printf("Bbox num:%d\n", cluster_size);

    for(int i=0;i<cluster_size;i++){
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.header.frame_id = "/velodyne";
        bbox.header.stamp = ros::Time::now();
        bbox.pose = cluster_array.clusters[i].pose;
        bbox.dimensions.x = cluster_array.clusters[i].depth;
        bbox.dimensions.y = cluster_array.clusters[i].width;
        bbox.dimensions.z = cluster_array.clusters[i].height;
        bbox.value = i;
        bbox_array.boxes.push_back(bbox);
    }
    pub_bbox.publish(bbox_array);
    pub_cluster.publish(cluster_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clustering");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/velodyne/normal_estimation", 10, pcCallback);

    pub_bbox = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/clustering/bbox", 10);
    pub_cluster = n.advertise<ssd_ros_msgs::ClusterArray>("/clustering", 10);

    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
