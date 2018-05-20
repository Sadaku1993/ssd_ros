#include <ssd_clustering/rm_normal_clustering_tf.h>

ros::Publisher pub_bbox;
ros::Publisher pub_cluster;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr cloud (new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    amsl_recog_msgs::ObjectInfoArray cluster_array; // changed by Y.S
    clustering(cloud, cluster_array);

    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    bbox_array.header.frame_id = target_frame;
    bbox_array.header.stamp = ros::Time::now();

    int cluster_size = int(cluster_array.object_array.size());  // changed by Y.S

    printf("Bbox num:%d\n", cluster_size);

    for(int i=0;i<cluster_size;i++){
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.header.frame_id = target_frame;
        bbox.header.stamp = ros::Time::now();
        bbox.pose         = cluster_array.object_array[i].pose;   //changed by Y.S
        bbox.dimensions.x = cluster_array.object_array[i].depth;
        bbox.dimensions.y = cluster_array.object_array[i].width;
        bbox.dimensions.z = cluster_array.object_array[i].height;

        bbox.value = i;
        bbox_array.boxes.push_back(bbox);
    }
    pub_bbox.publish(bbox_array);
    pub_cluster.publish(cluster_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rm_normal_clustering_tf");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/velodyne_points/rm_normal/tf", 10, pcCallback);

    pub_bbox = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/rm_normal/clustering/tf/bbox", 10);
    //pub_cluster = n.advertise<ssd_ros_msgs::ClusterArray>("/rm_normal/clustering/tf", 10);
    pub_cluster = n.advertise<amsl_recog_msgs::ObjectInfoArray>("/rm_normal/clustering/tf", 10);

    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
