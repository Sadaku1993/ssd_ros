#include <ros/ros.h>
#include "ros/package.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ssd_ros_msgs/BoundingBox.h>
#include <ssd_ros_msgs/BoundingBoxArray.h>
#include <ssd_ros_msgs/SSD.h>
#include <map>
#include <sensor_msgs/image_encodings.h>

using namespace std;

ssd_ros_msgs::BoundingBoxArrayConstPtr boxes_;
bool boxes_flag = false;
void boundingboxCallback(const ssd_ros_msgs::BoundingBoxArrayConstPtr& msg)
{
    boxes_ = msg;
    boxes_flag = true;
}

ssd_ros_msgs::BoundingBoxArray select_cluster(ssd_ros_msgs::BoundingBoxArray msg)
{
    ssd_ros_msgs::BoundingBoxArray data_array;
    for(size_t i=0;i<msg.boxes.size();i++){
        if(msg.boxes[i].Class=="person" || msg.boxes[i].Class=="car"){
            data_array.boxes.push_back(msg.boxes[i]);
        }
    }
    return data_array;
}

cv::Rect get_rect(ssd_ros_msgs::BoundingBox msg)
{
    cv::Rect rect;
    rect.x = msg.xmin;
    rect.y = msg.ymin;
    rect.width  = msg.xmax- msg.xmin;
    rect.height = msg.ymax - msg.ymin;
    return rect;
}

void overlap(const ssd_ros_msgs::BoundingBoxArrayConstPtr& msg)
{
    ssd_ros_msgs::BoundingBoxArray data_array = select_cluster(*msg);
    cout << "input:"<< msg->boxes.size() <<" pickup:" << data_array.boxes.size() << endl;

    if(0<data_array.boxes.size()){

        ssd_ros_msgs::BoundingBox data;
        std::vector<ssd_ros_msgs::BoundingBox> ssd_vector;

        for(size_t i=0;i<data_array.boxes.size();i++){
            ssd_vector.push_back(data_array.boxes[i]);
        }

        float matrix[data_array.boxes.size()][data_array.boxes.size()];
        
        for(size_t i=0;i<data_array.boxes.size();i++){
            for(size_t j=0;j<data_array.boxes.size();j++){
                cv::Rect rect1 = get_rect(data_array.boxes[i]);
                cv::Rect rect2 = get_rect(data_array.boxes[j]);
                cv::Rect overlap = rect1 & rect2;
                // cout<<"x:"<<overlap.x<<" y:"<<overlap.y<<" width:"<<overlap.width<<" height:"<<overlap.height<<endl;
                if(i==j){
                    matrix[i][j] = 0;
                }else{
                    matrix[i][j] = float(overlap.width*overlap.height)/(rect1.width*rect1.height);
                }
            }
        }

        
        for(size_t i=0;i<data_array.boxes.size();i++){
            for(size_t j=0;j<data_array.boxes.size();j++){
                printf ("%.2f ", matrix[i][j]);
            }
            printf("\n");
        }
    }
    boxes_flag = false;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "ssd_overlap");
    ros::NodeHandle n;

    ros::Subscriber box_sub   = n.subscribe("/zed/BoxArray",10,boundingboxCallback);

    ros::Rate rate(20);

    while(ros::ok())
    {
        if(boxes_flag){
            overlap(boxes_);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
