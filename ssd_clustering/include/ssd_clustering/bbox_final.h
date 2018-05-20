/*

author:Yudai Sadakuni

sensor fusion により取得したデータの可視化, 
トラッキング用のデータをPublish

*/

#include <ros/ros.h>
#include "ros/package.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

using namespace std;

string target_frame = "/zed_left_camera";
string source_frame = "/velodyne";

ros::Publisher pub;

void boundingbox(const amsl_recog_msgs::ObjectInfoArrayConstPtr msg)
{
    amsl_recog_msgs::ObjectInfoArrayConstPtr objects;
    objects = msg;

    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    bbox_array.header.frame_id = target_frame;
    bbox_array.header.stamp = ros::Time::now();

    for(int i=0;i<int(objects->object_array.size());i++){
        amsl_recog_msgs::ObjectInfoWithROI roi = objects->object_array[i];
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.header.frame_id = target_frame;
        bbox.header.stamp    = ros::Time::now();
        bbox.pose            = roi.pose;
        bbox.dimensions.x    = roi.depth;
        bbox.dimensions.y    = roi.width;
        bbox.dimensions.z    = roi.height;
        bbox.value           = i;
        bbox_array.boxes.push_back(bbox);
    }
    pub.publish(bbox_array);
}

