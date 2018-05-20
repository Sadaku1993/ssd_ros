#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>
#include<pcl/point_types.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <ssd_ros_msgs/Cluster.h>
#include <ssd_ros_msgs/ClusterArray.h>

#include "ssd_ros_tracking/ssd_ekf.h"

using namespace std;

// associatoin param
#define MIN_DIST 0.1
// dynamic detector param
#define CONFIDENCE 5

// erase flag param
#define MAX_P 20
#define MIN_CONF -5
#define MAX_RANGE 70

// カルマンフィルタパラメータ (要素数[0],[2]は並進速度, [1],[3])
double s_input[4];
// 観測
double s_measurement[3];

const double dt = 0.1;

bool clusters_flag = false;

ros::Publisher pub_centroid;
ros::Publisher pub_points;
ros::Publisher pub_track_vel;
ros::Publisher pub_centroid_test;
ros::Publisher pub_points_test;

float calcDistance(PointA p1, PointA p2)
{
	return sqrt((p1.x-p2.x) * (p1.x-p2.x) + (p1.y-p2.y) * (p1.y-p2.y));
}

void getClusterComponent(clusterInfo& clusters, CloudA& points, PointA& centroid)
{
    // size_t size = points.size();
    // clusters.comp.points.resize(size);
    clusters.comp = points;
}

void updateClassConfidence(vector<clusterInfo>& clusters, vector<bool>& update_list)
{
    int num_clusters = clusters.size();
    for (int i=0;i<num_clusters;i++)
    {
        // 同一物体として更新 -> clusters[i]の信頼度を高める
        if(update_list[i]){
            clusters[i].confidence++;
        }
        // 同一物体を検出できなかった場合 -> clusters[i]の信頼度を下げる
        else{
            clusters[i].confidence -= 2;
        }
    }
}

void checkClusterClass( vector<clusterInfo>& clusters, 
                        ssd_ros_msgs::ClusterArray ssd_clusters,
                        double *s_measurement)
{
    int ssd_num_position = ssd_clusters.clusters.size();    // ssdで認識したcluster
    int num_clusters = clusters.size();     // 保存しているcluster
    vector<bool> update_list(num_clusters);

    cout<<"size:"<<ssd_num_position<<endl;
    // ssdにて認識したクラスタ数分まわす
    for(int i=0;i<ssd_num_position;i++)
    {
        int associated_id = 0;
        float min_dist = MIN_DIST;
        bool update_flag = false;

        PointA centroid;
        CloudA centroid_;
        CloudA points;

        sensor_msgs::PointCloud2 pc2_centroid;
        sensor_msgs::PointCloud2 pc2_points;
        pc2_centroid = ssd_clusters.clusters[i].centroid;
        pc2_points   = ssd_clusters.clusters[i].points;

        pcl::fromROSMsg(pc2_centroid, centroid_);
        pcl::fromROSMsg(pc2_points, points);
        // pcl::fromROSMsg(ssd_clusters.clusters[i].centroid, centroid_);
        // pcl::fromROSMsg(ssd_clusters.clusters[i].points, points);
        centroid = centroid_.points[0];


        cout<<"check cluster"<<endl;
        for(size_t i=0;i<centroid_.points.size();i++)
        {
            cout<<"x : "<<centroid_.points[i].x<<" y : "<<centroid_.points[i].y<<endl;
        }
        cout<<"points size : "<<points.points.size()<<endl;

        // clusters内に格納されているデータと距離が近いか確認
        // 近ければ同一物体とみなすa
        // 最も距離の短いクラスタを同一とする
        for(int j=0;j<num_clusters; ++j){
            if(update_list[j] == false){
                if(centroid.x != 0 && centroid.y != 0)
                {
                    float dist = calcDistance(clusters[j].pre_position[0], centroid);
                    if(dist < min_dist){
                        associated_id = j;  // 格納されてるidを取得
                        min_dist = dist;
                        update_flag = true;
                    }
                }
            }
        }

        // 以前のclusters内に格納されているデータを同一のクラスタを検出できた場合
        if(update_flag){
            CloudA obj_comp;
            update_list[associated_id] = true;
            getClusterComponent(clusters[associated_id], points, centroid);

            // 観測を更新
            MeasurementUpdate(clusters[associated_id], centroid, dt, s_measurement);

            // 更新された場合、publish
            clusters[associated_id].update_comp_flag = true;
        }
        // 同一人物の可能性のあるクラスタが見つからない場合 -> 初期の観測としてclustersに格納
        else{
            if(centroid.x != 0 || centroid.y != 0)
            {
                int pre_size = clusters.size();
                // clustersのサイズを一つ大きくする(以前のデータは削除されない)
                clusters.resize(pre_size + 1);
                initCluster(clusters[pre_size], centroid);
                getClusterComponent(clusters[pre_size], points, centroid);
            }
        }
    }
    updateClassConfidence(clusters, update_list);
}

void pub_result( vector<clusterInfo> &clusters)
{
    CloudA ekf_centroid;
    CloudA ekf_points;
    sensor_msgs::PointCloud2 pc2_centroid;
    sensor_msgs::PointCloud2 pc2_points;

    int num_clusters = clusters.size();
    
    for(int i=0;i<num_clusters;i++){
        if(clusters[i].update_comp_flag){
            if(clusters[i].update_comp_flag || clusters[i].confidence < CONFIDENCE){
                PointA centroid;
                CloudA points;
                
                centroid.x = clusters[i].x(0);
                centroid.y = clusters[i].x(1);
                centroid.z = 0.5;
                points = clusters[i].comp;

                ekf_centroid.push_back(centroid);
                for(size_t i=0;i<points.size();i++)
                {
                    ekf_points.push_back(points[i]);
                }
            }
        }
        clusters[i].update_comp_flag = false;
    }

    cout<<"ekf centroid"<<ekf_centroid.points.size()<<endl;

    toROSMsg(ekf_centroid, pc2_centroid);
    pc2_centroid.header.frame_id = "map";
    pc2_centroid.header.stamp = ros::Time::now();
    pub_centroid.publish(pc2_centroid);

    toROSMsg(ekf_points, pc2_points);
    pc2_points.header.frame_id = "map";
    pc2_points.header.stamp = ros::Time::now();
    pub_points.publish(pc2_points);
}

void predictCluster(vector<clusterInfo>& clusters, double *s_input)
{
    int num_clusters = clusters.size();
    bool erase_flag = false;

    for(int i=0; i<num_clusters; ++i)
    {
        Prediction(clusters[i], dt, s_input);

		if (fabs(clusters[i].x(0)) > MAX_RANGE || fabs(clusters[i].x(1)) > MAX_RANGE){
                erase_flag = true;
        }
        else if(clusters[i].confidence < MIN_CONF){
            erase_flag = true;
        }

        if(erase_flag)
        {
            clusters.erase(clusters.begin()+i);
            erase_flag = false;
        }
    }
}

// ssd_ros_msgs::ClusterArray clusters_;
// void callback(const ssd_ros_msgs::ClusterArray &msg)
// {
//     clusters_ = msg;
//     clusters_flag = true;
// }

ssd_ros_msgs::ClusterArrayConstPtr clusters_;
void clusterCallback(const ssd_ros_msgs::ClusterArrayConstPtr& msg)
{
    printf("subscribe\n");
    clusters_ = msg;
    clusters_flag = true;
    size_t size = clusters_->clusters.size();
    cout<<"size:"<<size<<endl;
    for(size_t i=0;i<size;i++)
    {
        sensor_msgs::PointCloud2 centroid;
        sensor_msgs::PointCloud2 points;
        centroid = clusters_->clusters[i].centroid;
        points   = clusters_->clusters[i].points;
        centroid.header.frame_id ="/map";
        centroid.header.stamp = ros::Time::now();
        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();
        pub_centroid_test.publish(centroid);
        pub_points_test.publish(points);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_tracking_ekf");
    ros::NodeHandle n;
    // ros::NodeHandle pnh("~");

    ros::Subscriber sub = n.subscribe("/cluster/array/tf", 1, clusterCallback);
    pub_centroid = n.advertise<sensor_msgs::PointCloud2>("/ekf/centroid",1);
    pub_points   = n.advertise<sensor_msgs::PointCloud2>("/ekf/points",1);
    pub_track_vel    = n.advertise<visualization_msgs::MarkerArray>("/tracking/velocity", 1);
    pub_centroid_test = n.advertise<sensor_msgs::PointCloud2>("/test/ekf/centroid", 1);
    pub_points_test = n.advertise<sensor_msgs::PointCloud2>("/test/ekf/points", 1);

    vector<clusterInfo> clusters(0);

    // カルマンフィルタパラメータ
    s_input[0] = 0.001;
    s_input[1] = 0.005;
    s_input[2] = 1.0e-05;
    s_input[3] = 0.0005;

    // 観測
    s_measurement[0] = 100;
    s_measurement[1] = 100;
    s_measurement[2] = 100;

    ros::Rate rate(10);

    while(ros::ok())
    {
        if(clusters_flag){
            checkClusterClass(clusters, *clusters_, s_measurement);
            clusters_flag = false;
        }

        if(clusters.size()){
            pub_result(clusters);               // tracking結果をpublish
            predictCluster(clusters, s_input);  // 予測を更新
            visualizeArrowIn3D(pub_track_vel, clusters);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
