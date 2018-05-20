#include <ros/ros.h>
#include "ros/package.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ssd_ros_msgs/BoundingBox.h>
#include <ssd_ros_msgs/BoundingBoxArray.h>
#include <ssd_ros_msgs/SSD.h>
#include <map>
#include <sensor_msgs/image_encodings.h>

#ifndef INFINITY
#define INFINITY HUGE_VAL
#endif

#ifndef NAN
#define NAN 0xffc00000
#endif

using namespace std;
using namespace Eigen;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    cloud->width = (uint32_t) cloud->points.size();
    cloud->height = 1;

    float angularResolution = (float) ( 0.5f * (M_PI/180.0f));
    float maxAngleWidth     = (float) (360.f * (M_PI/180.0f));
    float maxAngleHeight    = (float) (180.f * (M_PI/180.0f));
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
            sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    std::cout << rangeImage << "\n";

    // レンジデータの最大値と最小値を取得
	float min_range, max_range;
	rangeImage.getMinMaxRanges(min_range, max_range);
	float min_max_range = max_range - min_range;

	// 結果画像領域を生成
	cv::Mat image(int(rangeImage.height), int(rangeImage.width), CV_8UC3);
    unsigned char r,g,b;
	for(int y=0; y < rangeImage.height; y++){
		for(int x=0; x<rangeImage.width; x++){
			pcl::PointWithRange rangePt = rangeImage.getPoint(x,y);
			if(rangePt.range == INFINITY || rangePt.range == -INFINITY || rangePt.range == NAN){
				// レンジが無限遠や無効なデータの場合
				pcl::visualization::FloatImageUtils::getColorForFloat (rangePt.range, r, g, b);
			}
			else{
				// レンジを0-1の範囲に正規化し、カラー値へ変換
				float value = (rangePt.range - min_range) / min_max_range;
				pcl::visualization::FloatImageUtils::getColorForFloat (value, r, g, b);
			}
			// ピクセルにカラー値を格納
			image.at<cv::Vec3b>(y,x)[0] = b;
			image.at<cv::Vec3b>(y,x)[1] = g;
			image.at<cv::Vec3b>(y,x)[2] = r;
		}
	}
	// 画像を保存
	// cv::imwrite("range_image.png", image);
    cv::imshow("image", image);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_range_image");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/velodyne_points/center", 10, pcCallback);
    
    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
