#include <ros/ros.h>
#include "ros/package.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ssd_ros_msgs/BoundingBox.h>
#include <ssd_ros_msgs/BoundingBoxArray.h>
#include <ssd_ros_msgs/SSD.h>
#include <map>
#include<sensor_msgs/image_encodings.h>
// #include<image_transport/image_transport.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Publisher pub_person;
ros::Publisher pub_people;
ros::Time t;

string target_frame = "/zed_left_camera";
string source_frame = "/velodyne";

bool pc_flag = false;
bool camera_flag = false;
bool image_flag = false;
bool boxes_flag = false;

sensor_msgs::PointCloud pc_;
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_);
    t = msg->header.stamp;
    pc_flag = true;
}

sensor_msgs::CameraInfoConstPtr camera_;
void cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    camera_ = msg;
    camera_flag = true;
}

sensor_msgs::ImageConstPtr image_;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    image_ = msg;
    image_flag = true;
}

ssd_ros_msgs::BoundingBoxArrayConstPtr boxes_;
void boundingboxCallback(const ssd_ros_msgs::BoundingBoxArrayConstPtr& msg)
{
    boxes_ = msg;
    boxes_flag = true;
}


void human_detection(sensor_msgs::PointCloud2 pcl_msg, const sensor_msgs::CameraInfoConstPtr& cinfo_msg, const sensor_msgs::ImageConstPtr& image_msg, const ssd_ros_msgs::BoundingBoxArrayConstPtr& boxes_msg)
{
    ROS_INFO("\n\nColouring VELODYNE CLOUD!!");

    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
      cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(image_msg)->image;
    // こっちのほうがいいのでは？？
    // cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;

    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudXYZRGB::Ptr coloured = PointCloudXYZRGB::Ptr(new PointCloudXYZRGB);

    fromROSMsg(pcl_msg, *trans_cloud);

    trans_cloud->header.frame_id = target_frame;

    pcl::copyPointCloud(*trans_cloud, *coloured);

    pcl::PointCloud<pcl::PointXYZRGB> pcl_person;
    std::map<int, pcl::PointCloud<pcl::PointXYZRGB> > pcl_people;

    int person_num = 0;
    for(int i=0;i<boxes_msg->boxes.size();++i)
    {
        const ssd_ros_msgs::BoundingBox &data = boxes_msg->boxes[i];
        ROS_INFO_STREAM("CLASS: "<< data.Class << " probability: "<< data.probability);
        if(data.Class=="person") person_num += 1;
    }
    printf("person num : %d\n",person_num);

    int count = 0;
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = coloured->points.begin(); pt < coloured->points.end(); ++pt)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);
        // if (count % 1000 == 0){
        //     printf("pt.x:%.2f pt.y:%.2f pt.z:%.2f\n", (*pt).x, (*pt).y, (*pt).z);
        //     printf("uv.x:%.2f uv.y:%.2f\n", uv.x, uv.y);
        // }
        for(int i=0;i<boxes_msg->boxes.size();++i)
        {
            const ssd_ros_msgs::BoundingBox &data = boxes_msg->boxes[i];
            if(data.Class=="person")
            {
                if(data.xmin<uv.x && uv.x<data.xmax && data.ymin<uv.y && uv.y<data.ymax){
                    pcl::PointXYZRGB p;
                    p.x = (*pt).x;
                    p.y = (*pt).y;
                    p.z = (*pt).z;
                    p.b = 50 * i;
                    p.g = 255 - 50*(i+1);
                    p.r = 50*(i+1);
                    
                    pcl::PointXYZRGB q;
                    q.x = (*pt).x;
                    q.y = (*pt).y;
                    q.z = (*pt).z;
                    q.b = image.at<cv::Vec3b>(uv)[0];
                    q.g = image.at<cv::Vec3b>(uv)[1];
                    q.r = image.at<cv::Vec3b>(uv)[2];


                    pcl_person.push_back(p);
                    pcl_people[i].push_back(q);
                }
            }
        }
        count++;
    }
    // ssdにて検出したperson領域内をpublish
    sensor_msgs::PointCloud2 pcl_person_ros;
    pcl::toROSMsg(pcl_person, pcl_person_ros);
    pcl_person_ros.header.stamp = t;
    pcl_person_ros.header.frame_id = target_frame;
    pub_person.publish(pcl_person_ros);

    // ラベル分割したpersonをpublish
    ssd_ros_msgs::SSD pcl_people_ros;
    for(auto it=pcl_people.begin();it!=pcl_people.end();++it)
    {
        sensor_msgs::PointCloud2 cloud_pc2;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_pcl = it->second;
        pcl::toROSMsg(cloud_pcl, cloud_pc2);
        cloud_pc2.header.stamp = t;
        cloud_pc2.header.frame_id = target_frame;
        pcl_people_ros.ssd_array.push_back(cloud_pc2);
    }
    pub_people.publish(pcl_people_ros);
    printf("Publish Obstaclenum : %d\n",pcl_people_ros.ssd_array.size());

}


int main(int argc,char** argv)
{
    ros::init(argc, argv, "ssd_human");
    ros::NodeHandle n;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    //ros::Subscriber pc_sub    = n.subscribe("/velodyne_points",10,pcCallback);
    // 三分割したvelodyneの点群をsubscribe
    ros::Subscriber pc_sub    = n.subscribe("/velodyne_points/center",10,pcCallback);
    ros::Subscriber cinfo_sub = n.subscribe("/zed/left/camera_info",10,cameraCallback);
    ros::Subscriber image_sub = n.subscribe("/zed/left/image_rect_color",10,imageCallback);
    ros::Subscriber box_sub   = n.subscribe("/zed/BoxArray",10,boundingboxCallback);

    pub_person  = n.advertise<sensor_msgs::PointCloud2> ("SSD/person",10);
    pub_people = n.advertise<ssd_ros_msgs::SSD> ("SSD/people",1000);

    ros::Rate rate(20);

    while(ros::ok())
    {
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;
        try{
            listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), t, ros::Duration(10.0));
            listener.transformPointCloud(target_frame.c_str(), t, pc_, source_frame.c_str(), pc_trans);
            sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
        }catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        }

        if(pc_flag && camera_flag && image_flag && boxes_flag){
            human_detection(pc2_trans, camera_, image_, boxes_);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
