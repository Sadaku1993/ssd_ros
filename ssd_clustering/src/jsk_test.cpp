#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBox.h>
// #include <visualization_tools/bounding_box.h>
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <jsk_recognition_msgs/SimpleOccupancyGrid.h>
#include <jsk_recognition_msgs/SimpleOccupancyGridArray.h>

#include <geometry_msgs/Point.h>

#include <ssd_ros_msgs/Points.h>

ros::Publisher pub_bbox;
ros::Publisher pub_pie_chart;

void pub_data(){
    jsk_recognition_msgs::BoundingBox bbox;
    bbox.header.frame_id = "velodyne";
    bbox.header.stamp = ros::Time::now();
    bbox.pose.position.x = 0;
    bbox.pose.position.y = 0;
    bbox.pose.position.z = 0.6;
    bbox.pose.orientation.x = 0;
    bbox.pose.orientation.y = 0;
    bbox.pose.orientation.z = 0;
    bbox.pose.orientation.w = 1;
    bbox.dimensions.x = 1.0;
    bbox.dimensions.y = 1.0;
    bbox.dimensions.z = 1.2;
    
    pub_bbox.publish(bbox);
    
    std_msgs::Float32 pie_chart;
    pie_chart.data = 0.2f;
    pub_pie_chart.publish(pie_chart);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jsk_test");
    ros::NodeHandle n;
    ros::Rate rate(20);
    pub_bbox      = n.advertise<jsk_recognition_msgs::BoundingBox>("/jsk/bbox", 10);
    pub_pie_chart = n.advertise<std_msgs::Float32>("/jsk/pie_chart", 10);

    geometry_msgs::Pose pose_bb;
    pose_bb.position.x = 0.0;
    pose_bb.position.y = 0.0;
    pose_bb.position.z = 0.0;
    pose_bb.orientation.x = 0.0;
    pose_bb.orientation.y = 0.0;
    pose_bb.orientation.z = 0.0;
    pose_bb.orientation.w = 1.0;

    // BoundingBox bb;
    // bb.setTopicName("/test/bb");
    // bb.setFrameId("velodyne");
    // bb.setSize(1.7, 0.5, 0.3); //height, width, depth
    // // bb.setCenter(0.0, 0.0, 0.0); //position (x, y, z)
    // bb.setPose(pose_bb);

    while(ros::ok())
    {
        pub_data();
        // bb.publish();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
