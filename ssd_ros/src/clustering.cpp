#include<ssd_ros/clustering.h>

bool pc_flag = false;

sensor_msgs::PointCloud2ConstPtr pc2_;
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pc2_ = msg;
    pc_flag = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clustering");
    ros::NodeHandle n;

    ros::Subscriber velodyne_callback = n.subscribe("/velodyne_points/center", 10, pcCallback);

    ros::Rate rate(20);
    
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}
