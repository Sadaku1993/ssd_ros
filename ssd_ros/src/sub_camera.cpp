#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>

using namespace std;

//ros::Publisher pub_image;

sensor_msgs::Image image_;
void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    image_ = *image_msg;
    //pub_image.publish(image_);
    
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvShare(image_msg,"bgr8");
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    // image = cv_bridge::toCvShare(image_msg)->image;

    cv::Mat image = cv_img_ptr->image;
    cv::imshow("SSD result", image);
    cv::waitKey(10);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sub_camera");
    ros::NodeHandle n;

    printf("subscribe camera data\n");

    ros::Subscriber sub_image = n.subscribe("usb_cam/image_raw",10,imageCallback);

    //pub_image = n.advertise<sensor_msgs::Image>("ssd_ros/image_raw",10);

    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
