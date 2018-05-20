#include "ssd_ros_tracking/ssd_ekf.h"
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

// dynamic detector param
#define CONFIDENCE 0

using namespace std;
// using namespace Eigen;

void visualizeArrowIn3D(ros::Publisher& pub, std::vector<clusterInfo>& clusters)
{
	int arrow_num = clusters.size();
	double yaw;
	double cluster_vel;
	double r = 0.0;
	double g = 0.0;
	double b = 0.0;

	visualization_msgs::MarkerArray arrows;
	visualization_msgs::Marker arrow;
	arrows.markers.resize(arrow_num);

	for (int i=0; i<arrow_num; i++){

		// Set the frame ID and timestamp.
		arrow.header.frame_id = "/zed_left_camera";
		arrow.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		arrow.ns = "velocity";
		arrow.id = i;

		// Set the marker type.
		arrow.type = visualization_msgs::Marker::ARROW;

		// Set the marker action.  Options are ADD and DELETE
		arrow.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.
		arrow.pose.position.x = clusters[i].x(0);
		arrow.pose.position.y = clusters[i].x(1);
		arrow.pose.position.z = -1.3;

		// yaw         = atan2(clusters[i].x(2),clusters[i].x(3));
		// yaw         = atan2(clusters[i].x(3),clusters[i].x(2));
		yaw         = clusters[i].x(2);
		cluster_vel = clusters[i].u(0);

		arrow.pose.orientation.x = 0.0;
		arrow.pose.orientation.y = 0.0;
		arrow.pose.orientation.z = sin(yaw*0.5);
		arrow.pose.orientation.w = cos(yaw*0.5);

		if(clusters[i].confidence < 3 || clusters[i].label == 0){ // label 0:static 1:dynamic
		// if (0.1 < cluster_vel && cluster_vel < 5){
			arrow.scale.x = 0.001;
			arrow.scale.y = 0.001;
			arrow.scale.z = 0.001;
		}
		else {
			arrow.scale.x = 0.4;
			arrow.scale.y = 0.4;
			arrow.scale.z = 0.1;
			// cout<<"clusters["<<i<<"].confidence = "<<clusters[i].confidence<<" label = "<<clusters[i].label<<endl;
		}

		arrow.color.r = r;
		arrow.color.g = g;
		arrow.color.b = b;
		arrow.color.a = 1.0;

		int init_num = -1;
		int pre_size = clusters[i].pre_position.size();
		// cout<<"pre_size = "<<pre_size<<endl;
		// cout<<"pre_size = "<<clusters[i].pre_position.size()<<endl;
		for (int j=0; j<pre_size; j++){
			// cout<<"label = "<<clusters[i].label<<endl;

			// if(clusters[i].confidence > (CONFIDENCE-1) && clusters[i].label == 1){ // label 0:static 1:dynamic

				if(init_num != i){
					cout<<"                      clusters["<<i<<"]                   "<<endl;
					// cout<<"i = "<<i<<" confidence = "<<clusters[i].confidence<<" now_pos["<<j<<"] x = "<<clusters[i].x(0)<<" y = "<<clusters[i].x(1)<<" label = "<<clusters[i].label<<endl;
					cout<<"i = "<<i<<" confidence = "<<clusters[i].confidence<<" now_pos["<<j<<"] x = "<<clusters[i].x(0)<<" y = "<<clusters[i].x(1)<<" vel = "<<clusters[i].velocity<<endl;
					init_num = i;
				}

				cout<<"i = "<<i<<" confidence = "<<clusters[i].confidence<<" pre_pos["<<j<<"] x = "<<clusters[i].pre_position[j].x<<" y = "<<clusters[i].pre_position[j].y<<endl;
			// }
		}
		
		arrow.lifetime    = ros::Duration(0.1);
		arrows.markers[i] = arrow;
	}

	cout<<"-------------------------------------------------------------"<<endl;


	// Publish the marker
	pub.publish(arrows);
}
