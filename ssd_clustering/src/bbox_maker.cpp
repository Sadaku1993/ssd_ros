/*

author:Yudai Sadakuni

点群のクラスタとSSDのクラスタを統合

*/

#define DEBUG 1
#define RESULT 1
#define SHOW 1

#include <ssd_clustering/bbox_maker.h>

string target_frame = "/zed_left_camera";
string source_frame = "/velodyne";

bool info_flag    = false;
bool image_flag   = false;
bool cluster_flag = false;
bool box_flag     = false;

// camera image callback
sensor_msgs::ImageConstPtr image_;
void image_Callback(const sensor_msgs::ImageConstPtr msg){
    image_ = msg;
    image_flag = true;
}

// camera info callback
sensor_msgs::CameraInfoConstPtr info_;
void info_Callback(const sensor_msgs::CameraInfoConstPtr msg){
    info_ = msg;
    info_flag = true;
}

// pointcloud cluster callback
amsl_recog_msgs::ObjectInfoArrayConstPtr clusters_;
void cluster_Callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr msg){
    clusters_ = msg;
    cluster_flag = true;
}

// ssd boundingbox callback
amsl_recog_msgs::ObjectInfoArrayConstPtr boxes_;
void boundingbox_Callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr msg){
    boxes_ = msg;
    box_flag = true;
}

// cluster and boundingbox fusion
void boundingbox_maker(const sensor_msgs::ImageConstPtr& image_msg,
                       const sensor_msgs::CameraInfoConstPtr& cinfo_msg,
                       const amsl_recog_msgs::ObjectInfoArrayConstPtr& cluster_msg,
                       const amsl_recog_msgs::ObjectInfoArrayConstPtr& bbox_msg){
    // convert from ros_image to opencv_image 
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
      cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);
    
    // DEBUG用のImgeを作成
    // cv::Mat image = cv::Mat::zeros(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;

    amsl_recog_msgs::ObjectInfoArray cluster_roi;
    amsl_recog_msgs::ObjectInfoArray bbox_roi;
    amsl_recog_msgs::ObjectInfoArray detect_roi;


    // ROI(pointcloud clustering)
    roi_for_cluster(cluster_msg, cam_model_, cluster_roi, image);
    int cluster_pick_up = int(cluster_roi.object_array.size());
    
    // ROI(object detector)
    roi_for_bbox(bbox_msg, bbox_roi, image);
    int bbox_pick_up    = int(bbox_roi.object_array.size()); 
    
    // ROI(sensor fusion)
    roi_for_fusion(bbox_roi, cluster_roi, detect_roi, image);
    int detect_pick_up = int(detect_roi.object_array.size());

    // Publish Data Assosiation Result
    pub.publish(detect_roi);

    
    // Visualize Rectangle on Image 
    if (DEBUG){
        char value_cluster[256];
        sprintf(value_cluster, "Cluster:%d", cluster_pick_up);
        cv::putText(image, value_cluster, cv::Point(20,100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2, CV_AA); 

        char value_bbox[256];
        sprintf(value_bbox, "BBox:%d", bbox_pick_up);
        cv::putText(image, value_bbox, cv::Point(20,150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2, CV_AA); 

        char value_detect[256];
        sprintf(value_detect, "Detect:%d", detect_pick_up);
        cv::putText(image, value_detect, cv::Point(20,200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,0), 2, CV_AA); 

        cv::imshow("DEBUG", image);
        cv::waitKey(1);
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "bbox_maker");
    ros::NodeHandle n;

    ros::Subscriber image_sub   = n.subscribe("/zed/left/image_rect_color", 10, image_Callback);
    ros::Subscriber info_sub    = n.subscribe("/zed/left/camera_info", 10, info_Callback);
    ros::Subscriber clusrer_sub = n.subscribe("/rm_normal/clustering/tf", 10, cluster_Callback);
    ros::Subscriber box_sub     = n.subscribe("/zed/BoxArray", 10, boundingbox_Callback); 
    
    pub  = n.advertise<amsl_recog_msgs::ObjectInfoArray>("/bbox_maker/ssd", 20);

    ros::Rate rate(20);
    while(ros::ok())
    {
        if(image_flag && info_flag && cluster_flag && box_flag){
            printf("ALL GREEN\n");
            boundingbox_maker(image_, info_, clusters_, boxes_);
            image_flag   = false;
            info_flag    = false;
            cluster_flag = false;
            box_flag    = false;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

