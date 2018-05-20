#include<ros/ros.h>
#include<ssd_ros_msgs/BoundingBox.h>
#include<ssd_ros_msgs/BoundingBoxArray.h>

using namespace std;

void boxCallback(const ssd_ros_msgs::BoundingBoxArrayConstPtr& msg)
{
    for(int i=0;i<msg->boxes.size(); i++)
    {
        const ssd_ros_msgs::BoundingBox &data = msg->boxes[i];
        ROS_INFO_STREAM("CLASS: "<< data.Class << " probability: "<< data.probability);
        // ROS_INFO_STREAM("UL: " << data.upperleft << "UR: " << data.upperright <<
        //              "color: " << data.color << "ID: " << data.cameraID);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sub_boundingbox");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/BoxArray",10,boxCallback);

    ros::Rate rate(30);
    
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
