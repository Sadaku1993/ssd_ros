#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

struct Position{
	double x;
	double y;
	double yaw;
};

// typedef struct{
// 	int count;
// 	int confidence; // when human is detected, transition +1, else -1 
//     int label;		// Static or Dynamic
//     int color_id;
// 	bool update_flag;
// 	bool update_comp_flag;
// 	bool init_flag;
// 	float width;
// 	float height;
// 	float length;
// 	float likelihood;
// 	float track_num;
// 	float r;
// 	float yaw;
// 	double dx;
// 	double dy;
// 	double velocity;
// 	geometry_msgs::Point32 pre_vel; // previous velocity 
// 	// ekfでの予測値
//     Eigen::Vector3f x;				// state for filter
// 	Eigen::Matrix3f P;				// uncertainity 
// 	Eigen::Vector2f u;				// (0,0,accel_x,accel_y) 
// 	Eigen::Matrix2f R;				// measurement uncertainty
// 	// std::vector<pcl::PointXYZINormal> pre_position;//pre_position
//     std::vector<pcl::PointXYZINormal> pre_position;//pre_position
//     Position global_position;
// 	pcl::PointCloud<pcl::PointXYZINormal> comp;//cluster component
// }clusterInfo;

typedef struct{
	int count;
	int confidence; // when human is detected, transition +1, else -1 
    int label;		// Static or Dynamic
    int color_id;
	bool update_flag;
	bool update_comp_flag;
	bool init_flag;
	float width;
	float height;
	float length;
	float likelihood;
	float track_num;
	float r;
	float yaw;
	double dx;
	double dy;
	double velocity;
	geometry_msgs::Point32 pre_vel; // previous velocity 
    Eigen::Vector3f x;				// state for filter
	Eigen::Matrix3f P;				// uncertainity 
	Eigen::Vector2f u;				// (0,0,accel_x,accel_y) 
	Eigen::Matrix2f R;				// measurement uncertainty
    std::vector<pcl::PointXYZRGB> pre_position;//pre_position
    Position global_position;
	pcl::PointCloud<pcl::PointXYZRGB> comp;//cluster component
}clusterInfo;


class EKF{
private:
public:
	MatrixXf move(MatrixXf x, MatrixXf u, float dt);
	MatrixXf jacobF(MatrixXf x, MatrixXf u, float dt);
	MatrixXf jacobG(MatrixXf x, MatrixXf u, float dt);
	MatrixXf jacobV(MatrixXf x, MatrixXf u, float dt);
	MatrixXf jacobM(MatrixXf u, double *s_input);
	MatrixXf jacobH(MatrixXf x);
};

typedef pcl::PointXYZRGB PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

// void setParameter(KalmanFilter& kf, double dt);
void initCluster(clusterInfo& cluster, PointA& init_pt);
void Prediction( clusterInfo& cluster,
				 double dt,
				 double *s_input);
void MeasurementUpdate(	clusterInfo& cluster,
						PointA& obj_centroid,
						double dt,
						double *s_measurement);
// void visualizeArrowIn2D(ros::Publisher& pub, std::vector<clusterInfo>& clusters);
void visualizeArrowIn3D(ros::Publisher& pub, std::vector<clusterInfo>& clusters);
// void visualizeErrorEllipse(ros::Publisher& ee_pub, std::vector<clusterInfo>& clusters);
// void visualize_tracking_trajectory(ros::Publisher& trajectory_pub, std::vector<clusterInfo>& clusters);
