/*

author:Yudai Sadakuni

sensor fusion により取得したデータの可視化, 
トラッキング用のデータをPublish

*/

#include <ssd_clustering/bbox_final.h>

int flag = false;

amsl_recog_msgs::ObjectInfoArrayConstPtr bbox_;
void Callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr msg)
{
    bbox_ = msg;
    flag = true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bbox_final");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/bbox_maker/ssd", 20, Callback);
    pub = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bbox_final/ssd", 20);

    ros::Rate rate(20);

    while(ros::ok()){
        
        if(flag) boundingbox(bbox_);

        ros::spinOnce();
        rate.sleep();
    }
}
