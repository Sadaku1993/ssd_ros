#define DEBUG 1

#include "ssd_ros/velo2cam_utils.h"
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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace sensor_msgs;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
ros::Publisher pcl_pub;
//string target_frame, source_frame;
image_transport::Publisher pub;

typedef struct {
   double r,g,b;
} COLOUR;

COLOUR GetColour(double v,double vmin,double vmax)
{
   COLOUR c = {1.0,1.0,1.0}; // white
   double dv;

   if (v < vmin)
   v = vmin;
   if (v > vmax)
   v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return(c);
}

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

void
transformAsMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat)
{
  double mv[12];
  bt.getBasis ().getOpenGLSubMatrix (mv);

  tf::Vector3 origin = bt.getOrigin ();

  out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
  out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
  out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];

  out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
  out_mat (0, 3) = origin.x ();
  out_mat (1, 3) = origin.y ();
  out_mat (2, 3) = origin.z ();
}

void callback(const PointCloud2::ConstPtr& pcl_msg, const CameraInfoConstPtr& cinfo_msg, const ImageConstPtr& image_msg){
  if(DEBUG) ROS_INFO("Projecting poincloud to the image %ld", pcl_msg->header.stamp.toNSec()-image_msg->header.stamp.toNSec());

  // Conversion
  cv::Mat image;
  try{
    image = cv_bridge::toCvShare(image_msg)->image;
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(cinfo_msg);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); // From ROS Msg
  //pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>); // After transformation
  pcl::PointCloud<Velodyne::Point>::Ptr pcl_cloud(new pcl::PointCloud<Velodyne::Point>); // From ROS Msg
  pcl::PointCloud<Velodyne::Point>::Ptr trans_cloud(new pcl::PointCloud<Velodyne::Point>); // After transformation
  fromROSMsg(*pcl_msg, *pcl_cloud);

  Velodyne::addRange(*pcl_cloud);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  //cout << "FRAME ID "<< pcl_cloud->header.frame_id << endl;
  // pcl_ros::transformPointCloud("stereo_camera", *pcl_cloud, *trans_cloud, listener);

  if(DEBUG) ROS_INFO("Waiting for TF");

  try{
    // listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), ros::Time(0), ros::Duration(20.0));
    // listener.lookupTransform (target_frame.c_str(), source_frame.c_str(), ros::Time(0), transform);
    listener.waitForTransform(image_msg->header.frame_id.c_str(), pcl_msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(20.0));
    listener.lookupTransform (image_msg->header.frame_id.c_str(), pcl_msg->header.frame_id.c_str(), ros::Time(0), transform);
  }catch (tf::TransformException& ex) {
    ROS_WARN("[pcl_projection] TF exception:\n%s", ex.what());
    return;
  }

  if(DEBUG) ROS_INFO("TF available");
  Eigen::Matrix4f transf_mat;
  transformAsMatrix(transform, transf_mat);
  pcl::transformPointCloud(*pcl_cloud, *trans_cloud, transf_mat);

  //pcl_ros::transformPointCloud (*pcl_cloud, *trans_cloud, transform);
  trans_cloud->header.frame_id = image_msg->header.frame_id;

  //pcl::copyPointCloud(*trans_cloud, *coloured);

  cv::Mat alpha(image.size(), CV_8UC3);
  cv::Mat nice(image.size(), CV_8UC3);

  if(DEBUG) ROS_INFO("Projecting every single point. Please wait. ");

  #pragma omp parallel for
  for (pcl::PointCloud<Velodyne::Point>::iterator pt = trans_cloud->points.begin(); pt < trans_cloud->points.end(); pt++)
  {
    cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
    cv::Point2d uv;
    uv = cam_model_.project3dToPixel(pt_cv);

    //int intensity = clip<int>(int((*pt).range/20.0*255.0), 0, 255);

    //cv::circle(image, uv, 4, cv::Scalar(intensity, intensity, intensity), -1);

    if((*pt).z>0 && uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
      COLOUR c = GetColour(int((*pt).range/20.0*255.0), 0, 255);

      cv::circle(image, uv, 3, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);

      cv::addWeighted(alpha, 0.5, image, 1, 0, nice);
    }

    //if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
    // Copy colour to laser pointcloud
    //(*pt).b = image.at<cv::Vec3b>(uv)[0];
    //(*pt).g = image.at<cv::Vec3b>(uv)[1];
    //(*pt).r = image.at<cv::Vec3b>(uv)[2];
    //}

  }

  pub.publish(cv_bridge::CvImage(image_msg->header, image_msg->encoding, image).toImageMsg());
  if(DEBUG) ROS_INFO("Done");
  //ROS_INFO("Publish coloured PC");

  // Publish coloured PointCloud
  /*sensor_msgs::PointCloud2 pcl_colour_ros;
  pcl::toROSMsg(*coloured, pcl_colour_ros);
  pcl_colour_ros.header.stamp = pcl_msg->header.stamp ;
  pcl_pub.publish(pcl_colour_ros);*/
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pcl_projection");
  ros::NodeHandle nh_("~"); // LOCAL
  image_transport::ImageTransport it(nh_);

  // Parameters
  //nh_.param<std::string>("target_frame", target_frame, "/stereo_camera");
  //nh_.param<std::string>("source_frame", source_frame, "/velodyne");

  // Subscribers
  message_filters::Subscriber<PointCloud2> pc_sub(nh_, "pointcloud", 1);
  message_filters::Subscriber<CameraInfo> cinfo_sub(nh_, "camera_info", 1);
  message_filters::Subscriber<Image> image_sub(nh_, "image", 1);

  //pcl_pub = nh_.advertise<PointCloud2> ("image_projected", 1);

  pub = it.advertise("image_with_points", 1);

  typedef message_filters::sync_policies::ApproximateTime<PointCloud2, CameraInfo, Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), pc_sub, cinfo_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();
  return 0;
}
