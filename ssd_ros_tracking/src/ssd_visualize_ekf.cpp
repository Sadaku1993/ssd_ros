#include "ssd_ros_tracking/ssd_ekf.h"
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

// dynamic detector param
#define CONFIDENCE 0

using namespace std;
// using namespace Eigen;

void coloring(int color_id, double& r, double& g, double& b)
{
	float colors[10][3] = {{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}, {255,255,255}, {128,0,128}, {128,128,128}, {128,128,0}};
	// float colors[10][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}, {0, 1, 1}, {1, 0, 1}, {1, 1, 1}, {0.5, 0, 0.5}, {0.5, 0.5, 0.5}, {0.5, 0.5, 0}};

	r = colors[color_id % 10][0];
	g = colors[color_id % 10][1];
	b = colors[color_id % 10][2];
}

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
		arrow.header.frame_id = "/centerlaser_";
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

		coloring(clusters[i].color_id, r, g, b);

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

// void visualizeErrorEllipse(ros::Publisher& pub, std::vector<clusterInfo>& clusters)/*{{{*/
// {
// 	float chi = 5.99;//97%:7.01;//99%:9.21
// 	float a=0, b=0, rot_angle = 0;
// 	int big_id = 1, small_id = 0, ee_num = 200;
// 	Vector2f e_val;
// 	MatrixXf e_vec;
// 	sensor_msgs::PointCloud ees; //ErroEllipses
// 	ees.header.frame_id = "/centerlaser_"; 
// 	// ees.header.frame_id = "centerlaser_"; 
// 	ees.header.stamp = ros::Time::now();
// 	ees.points.resize(clusters.size()*ee_num);
// 	// ees.points.resize(ee_num);//the number of points constituting ellipse
//
// 	if (clusters.size()){
// 		for (size_t id=0; id<clusters.size(); ++id){
// 			//get long length and short length of ellipse
// 			Eigen::SelfAdjointEigenSolver<Matrix2f> es(clusters[id].P.block(0,0,2,2));
// 			e_val = es.eigenvalues(); //get eigenvalues
// 			e_vec = es.eigenvectors(); //get eigenvectors
// 			if (e_val(0)>=e_val(1)){
// 				big_id = 0;
// 				small_id = 1;
// 			}
// 			else {
// 				big_id = 1;
// 				small_id = 0;
// 			}
// 			// a = sqrt(e_val(big_id)*chi);
// 			// b = sqrt(e_val(small_id)*chi);
// 			a = 0.8;
// 			b = 0.8;
// 			clusters[id].r=a;
//
// 			//large ellipse is not drawn
// 			// if (clusters[id].r<MAX_R){
// 			//ellipse : center(0,0)
// 			float theta = 0, r=0;
// 			for (int i=0; i<ee_num; ++i){
// 				theta =(float)((2.0 * M_PI)/ee_num)*i;
// 				r = b * b / (a + sqrt(a*a-b*b));
// 				ees.points[i+id*ee_num].x = r * cos(theta);
// 				ees.points[i+id*ee_num].y = r * sin(theta);
// 				ees.points[i+id*ee_num].z = -1.3;
// 			}
// 			//rotate and shift ellipse
// 			rot_angle = atan2(e_vec(1,big_id), e_vec(0,big_id));	
// 			for (size_t i=id*ee_num; i<(id+1)*ee_num; i++){
// 				float temp_x = ees.points[i].x;
// 				ees.points[i].x = temp_x * cos(rot_angle) + ees.points[i].y * sin(rot_angle) + clusters[id].x(0);
// 				ees.points[i].y = -temp_x * sin(rot_angle) + ees.points[i].y * cos(rot_angle)+ clusters[id].x(1);
// 			}
// 			// }
// 		}
// 	}
// 	else {
// 		ees.points.resize(1);	
// 		ees.points[0].x=0;	
// 		ees.points[0].y=0;	
// 		ees.points[0].z=-1000;	
// 	}
// 	pub.publish(ees);
// }/*}}}*/

// void visualize_tracking_trajectory(ros::Publisher& pub, std::vector<clusterInfo>& clusters)/*{{{*/
// {
// 	int clusters_num = clusters.size();
// 	double r = 0.0;
// 	double g = 0.0;
// 	double b = 0.0;
//
// 	visualization_msgs::Marker point, line_strip;
// 	visualization_msgs::MarkerArray points, line_strips;
//
// 	points.markers.resize(clusters_num);
// 	line_strips.markers.resize(clusters_num);
//
// 	for (int i=0; i<clusters_num; i++){
//
// 		point.header.frame_id = line_strip.header.frame_id = "/centerlaser_";
// 		point.header.stamp = line_strip.header.stamp = ros::Time::now();
// 		point.ns = line_strip.ns = "trajectory";
// 		point.action = line_strip.action = visualization_msgs::Marker::ADD;
// 		point.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
//
// 		point.id = i;
// 		line_strip.id = i + clusters_num;
//
// 		point.type		= visualization_msgs::Marker::POINTS;
// 		line_strip.type = visualization_msgs::Marker::LINE_STRIP;
//
// 		point.scale.x = 0.2;
// 		point.scale.y = 0.2;
//
// 		line_strip.scale.x = 0.1;
//
// 		// Points are green
// 		// point.color.g = 1.0f;
// 		// point.color.a = 1.0f;
// 		coloring(i, r, g, b);
// 		point.color.r = r;
// 		point.color.g = g;
// 		point.color.a = b;
//
// 		// Line strip is blue
// 		line_strip.color.b = 1.0;
// 		line_strip.color.a = 1.0;
//
// 		int pre_size = clusters[i].pre_position.size();
//
// 		for (int j=0; j<pre_size; j++){
// 			geometry_msgs::Point p;
//
// 			if (clusters[i].pre_position[j].x != 0 && clusters[i].pre_position[j].y != 0){
// 				p.x = clusters[i].pre_position[j].x;
// 				p.y = clusters[i].pre_position[j].y;
// 				p.z = -1.3;
// 				// cout<<"pre_pos["<<j<<"] x = "<<clusters[i].pre_position[j].x<<" y = "<<clusters[i].pre_position[j].y<<endl;
// 			}
//
// 			point.points.push_back(p);
// 			line_strip.points.push_back(p);
// 		}
//
// 		// point.lifetime		= ros::Duration(0.1);
// 		point.lifetime		= ros::Duration(1.0);
// 		line_strip.lifetime	= ros::Duration(0.1);
//
// 		// cout<<" ----------------> array"<<endl;
//
// 		points.markers[i] 	   = point;
// 		line_strips.markers[i] = line_strip;
//
// 	}
//
// 	pub.publish(points);
// 	// pub.publish(line_strips);
// }/*}}}*/
