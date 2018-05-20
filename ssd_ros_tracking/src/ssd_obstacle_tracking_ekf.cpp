#include "ssd_ros_tracking/ssd_ekf.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// erase flag param
#define MAX_P 20
#define MIN_CONF -5
#define MAX_RANGE 70

// association param
#define DIFF_W 1.0 
#define DIFF_H 1.0
#define DIFF_L 1.0
// #define MIN_DIST 0.5 // 2.5 [m/s]   時速 9km
#define MIN_DIST 2

// dynamic detector param
#define CONFIDENCE 5

int  color_id = 0;
int  start_index = 0;

bool using_colors[10];
bool init_flag1 = true;
bool init_flag2 = true;

float sum;
float before;

double qr;
double qp;
double qy;
double init_yaw;
double robot_pre_yaw;

double cluster_r;
double cluster_yaw;
double pre_yaw = 0.0;

double robot_velocity;
double robot_angular = 0;

double s_input[4]; 		// 制御の誤差パラメータ (要素数[0],[2]は並進速度，[1],[3]は回頭速度のパラメータ)
double s_measurement[3];

bool callback_imu  = false;
bool callback_odom = false;
bool callback_cluster_pos = false;

const double dt = 0.2;

CloudI points; 
CloudI centroid; 

using namespace std;

inline float calcDistance(PointI p1, PointI p2)
{
	return sqrt((p1.x-p2.x) * (p1.x-p2.x) + (p1.y-p2.y) * (p1.y-p2.y));
}

inline bool compareSize(clusterInfo& p1, PointI& p2)
{
	if (fabs(p2.normal_z-p1.height) < DIFF_H
	 && fabs(p2.normal_x-p1.width)  < DIFF_W
	 && fabs(p2.normal_y-p1.length) < DIFF_L)
		return true;

	else return false;
}

inline void getClusterComponent(clusterInfo& clusters, CloudI& obj, PointI& obj_info)
{
	int num_comp = obj_info.curvature;  //curvatureに点群数を格納してある
	clusters.comp.points.resize(num_comp);
	for (int j=0; j<num_comp; ++j) clusters.comp.points[j] = obj.points[j + obj_info.intensity];    // intensityに点群のindexが格納されている

    // 使ってないのでは??
	start_index += num_comp;
}


inline void updateClassConfidence( vector<clusterInfo>& clusters,
								   vector<bool>& update_list)
{
	int num_clusters = clusters.size();

	for (int i=0; i<num_clusters; i++){

		if (update_list[i]) clusters[i].confidence++; // clusters[i]の信頼度を高める

		else {
			// if (clusters[i].confidence > 0) clusters[i].confidence = 0; // clusters[i]の信頼度を0にする
			// else clusters[i].confidence--;
			clusters[i].confidence -= 2;
		}
	}
    // 色情報を付与するならここかな
}


inline void checkClusterClass( vector<clusterInfo>& clusters,
							   CloudI& cluster_points, 
							   CloudI& cluster_centroid,
							   double *s_measurement)
{
	int num_position = cluster_centroid.points.size();
	int num_clusters = clusters.size();
	vector<bool> update_list(num_clusters); // clustersの数だけupdata_listを作る


	for (int i=0; i<num_position; ++i){ // 人認識された数回す
		int associated_id = 0;
		float min_dist = MIN_DIST;

		bool update_flag = false;

		for (int j=0; j<num_clusters; ++j){ // clustersの数だけ比較

			if (update_list[j] == false){ // 同一人物の可能性が高いクラスタの抽出 -> updata_listに登録
				if(cluster_centroid.points[i].x != 0 || cluster_centroid.points[i].y != 0){

					float dist = calcDistance(clusters[j].pre_position[0], cluster_centroid.points[i]);

                    // 色情報を付与するならここかな
                    // 距離情報で付近にあれば同一物体として扱う (min_dist=0.5)
					if (dist < min_dist){
						associated_id = j;
						min_dist = dist;
						update_flag = true; // 同一人物の特定ができたらtrue
					}
				}
			}
		}

        // 以前のクラスタ内に同一のクラスタを検出できた場合
		if (update_flag){ // 上記の比較をパスするクラスタの決定
			CloudI obj_comp;
			update_list[associated_id] = true;  //associated_idのupdate_listは同一物体として観測されている
			getClusterComponent(clusters[associated_id], cluster_points, cluster_centroid.points[i]); // clustersに点群を格納

            // clusters[] : 事前に獲得したcluster[]情報
            // cluster_centroid.points[i] : 今回観測した重心情報
            // cluster[] : に今回の更新情報を格納
			MeasurementUpdate(clusters[associated_id], cluster_centroid.points[i], dt, s_measurement); // 観測を更新

            // 更新された場合、rvizに表示
			clusters[associated_id].update_comp_flag = true;
		}

		else { // 同一人物の可能性があるクラスタが見つからない場合 -> 初期の観測と考えclustersに格納

			if(cluster_centroid.points[i].x != 0 || cluster_centroid.points[i].y != 0){ //ノイズフィルタリング
				// 以前のクラスタサイズ
                int pre_size = clusters.size();
                
                // 新しく観測されたクラスタを格納するためサイズを一つup 
                //clustersの末尾に配列を１つ追加 (データはさくじょされない)
				clusters.resize(pre_size + 1);

                //新しく末尾に追加したcluster[]にcluster情報を追加
				initCluster(clusters[pre_size], cluster_centroid.points[i]); // clustersにクラスタを追加
				clusters[pre_size].color_id = color_id; // 色の決定
				using_colors[color_id] = true;

				getClusterComponent(clusters[pre_size], cluster_points, cluster_centroid.points[i]); // clusters.compに点群を格納

				for(int j=0; j<10; j++){
					if(using_colors[j] == false){
						color_id = j;
						using_colors[j] = true;
						break;
					}
				}
			}
		}
	}

	updateClassConfidence(clusters, update_list); // クラスタの信頼度の計算
	start_index = 0;
}

inline void predictCluster(vector<clusterInfo>& clusters, double *s_input)
{
	int  num_clusters = clusters.size();
	bool erase_flag   = false;

	for (int i=0; i<num_clusters; ++i){
		Prediction(clusters[i], dt, s_input);

		if (fabs(clusters[i].x(0)) > MAX_RANGE || fabs(clusters[i].x(1)) > MAX_RANGE){
			// if(clusters[i].confidence > (CONFIDENCE-1) && clusters[i].label == 1) cout<<"out of range x = "<<clusters[i].x(0)<<" y = "<<clusters[i].x(1)<<endl;
			erase_flag = true;
		}

		// else if (clusters[i].P(0, 0) >= MAX_P){ 
		// 	cout<<"too large R: "<<clusters[i].P(0, 0)<<endl;
		// 	erase_flag = true;
		// }
        //
        //
        
        //confidence(信頼値)がMIN_CONFを下回った場合, clustersからデータを削除する
		else if (clusters[i].confidence < MIN_CONF){
			// if(clusters[i].confidence > (CONFIDENCE-1) && clusters[i].label == 1) cout<<"due to likelihood"<<endl;
			erase_flag = true;
		}

		if (erase_flag){
			// cout<<"i = "<<i<<" eliminate "<<endl;	
			using_colors[clusters[i].color_id] = false;
			clusters.erase(clusters.begin()+i); //メモリの開放 開放された場所は詰められる

			erase_flag = false;
		}
	}
}

inline void pubPointCloud2(	ros::Publisher& pub,
							pcl::PointCloud<pcl::PointXYZINormal>& pcl_pt,
							const char* frame_id)
{
	sensor_msgs::PointCloud2 ros_pt;
	toROSMsg(pcl_pt, ros_pt);
	ros_pt.header.frame_id=frame_id;
	ros_pt.header.stamp=ros::Time::now();
	pub.publish(ros_pt);
}

inline void deletePointsInRviz( CloudI& tmp)
{
	tmp.points.resize(1);
	tmp.points[0].x         = -1000;
	tmp.points[0].y         = -1000;
	tmp.points[0].z         = -1000;
	tmp.points[0].intensity = 1;
}

inline void pub_result( vector<clusterInfo>& clusters,
						ros::Publisher dynamic_pub,
						ros::Publisher static_pub,
						ros::Publisher track_pub)
{
	CloudI dynamic_pt, static_pt, track_pt;
	if(clusters.size()){
		int pre_s = 0;
		int pre_d = 0;
		int num_clusters = clusters.size();
		track_pt.points.resize(num_clusters);

		for(int i=0; i<num_clusters; i++){

			if(clusters[i].update_comp_flag){
				track_pt.points[i].x = clusters[i].x(0);
				track_pt.points[i].y = clusters[i].x(1);
				track_pt.points[i].z = 1.0;

				int num_comp = clusters[i].comp.points.size();

                // confidence(信頼値) label(動的か静的か)によりpublishするかしないかを算出している
				if(clusters[i].confidence < CONFIDENCE || clusters[i].label == 0){ // label 0:static 1:dynamic
					static_pt.points.resize(pre_s + num_comp);

					for(int j=0; j<num_comp; j++){
						static_pt.points[j+pre_s]  = clusters[i].comp.points[j];
					}

					pre_s += num_comp;
				}

				else{
					dynamic_pt.points.resize(pre_d + num_comp);

					for(int j=0; j<num_comp; j++){
						dynamic_pt.points[j+pre_d] = clusters[i].comp.points[j];
					}
					pre_d += num_comp;
				}
			}
			clusters[i].update_comp_flag = false;
		}
		pubPointCloud2( static_pub,  static_pt, "odom");
		pubPointCloud2(dynamic_pub, dynamic_pt, "odom");
		pubPointCloud2(  track_pub,   track_pt, "odom");

		static_pt.points.erase(static_pt.points.begin(), static_pt.points.begin()+static_pt.points.size());
		dynamic_pt.points.erase(dynamic_pt.points.begin(), dynamic_pt.points.begin()+dynamic_pt.points.size());
	}
}

inline void ClusterPointsCallback( const sensor_msgs::PointCloud2::Ptr &msg)
{
	pcl::fromROSMsg(*msg, points);
}

inline void ClusterPositionCallback( const sensor_msgs::PointCloud2::Ptr &msg)
{
	pcl::fromROSMsg(*msg, centroid);
	callback_cluster_pos = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_tracking_ekf");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	ros::Subscriber cluster_pt_sub  = nh.subscribe("/dynamic_cluster/points/lcl", 1, ClusterPointsCallback);
	ros::Subscriber cluster_pos_sub = nh.subscribe("/dynamic_cluster/centroid/lcl", 1, ClusterPositionCallback);
    ros::Publisher track_pos_pub    = nh.advertise<sensor_msgs::PointCloud2>("/tracking/position", 1);
	ros::Publisher track_pt_pub     = nh.advertise<sensor_msgs::PointCloud2>("/tracking/points", 1);
	ros::Publisher track_vel_pub    = nh.advertise<visualization_msgs::MarkerArray>("/tracking/velocity", 1);
	ros::Publisher dynamic_pub      = nh.advertise<sensor_msgs::PointCloud2>("/dynamic/points", 1);
	ros::Publisher static_pub       = nh.advertise<sensor_msgs::PointCloud2>("/static/points", 1);
	ros::Publisher track_pub        = nh.advertise<sensor_msgs::PointCloud2>("/track/points", 1);
	ros::Publisher points_pub       = nh.advertise<sensor_msgs::PointCloud2>("/test/points", 1);
	
	vector<clusterInfo> clusters(0);

	ros::Rate loop_rate(5);

    // 速度ベクトルの色用
	for(int i=0; i<10; i++) using_colors[i] = false;

    // カルマンフィルタパラメータ
	s_input[0] = 0.001;
	s_input[1] = 0.005;
	s_input[2] = 1.0e-05;
	s_input[3] = 0.0005;

    // 観測 100がいい感じ
	s_measurement[0] = 100;
	s_measurement[1] = 100;
	s_measurement[2] = 100;

	cout<<"start"<<endl;

	while (ros::ok()){
	
		if (callback_cluster_pos){

			checkClusterClass(clusters, points, centroid, s_measurement);
			callback_cluster_pos=false;
		}

		if (clusters.size()){

			pub_result(clusters, dynamic_pub, static_pub, track_pub);
			predictCluster(clusters, s_input);
			visualizeArrowIn3D(track_vel_pub, clusters);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	cout<<"end"<<endl;	
	return 0;
}
