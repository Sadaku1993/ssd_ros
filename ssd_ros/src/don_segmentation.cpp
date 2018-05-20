#include<ssd_ros/don_segmentation.h>

using namespace std;
using namespace Eigen;

double scale1    = 0.1;
double scale2    = 0.3;
double threshold = 0.2;

ros::Publisher pub;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    //Downsample
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud (cloud);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.filter (*ds_cloud);

    // Create a search tree, use KDTree for non-organized data
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;

    if (ds_cloud->isOrganized ())
    {
        tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> ());
    }
    else
    {
        tree.reset (new pcl::search::KdTree<pcl::PointXYZRGB> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud (ds_cloud);

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
    ne.setInputCloud (ds_cloud);
    ne.setSearchMethod (tree);

    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    // calculate normals with the small scale
    cout << "Calculating normals for scale..." << scale1 << endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (scale1);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    cout << "Calculating normals for scale..." << scale2 << endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (scale2);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointNormal>(*ds_cloud, *doncloud);

    cout << "Calculating DoN... " << endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud (ds_cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*doncloud);

    // Filter by magnitude
    cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
            new pcl::ConditionOr<pcl::PointNormal> ()
            );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
                new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
            );
    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
    condrem.setInputCloud (doncloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

    // Apply filter
    condrem.filter (*doncloud_filtered);

    doncloud = doncloud_filtered;

    // Save filtered output
    std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;
    
    sensor_msgs::PointCloud2 pc2_output;
    pcl::toROSMsg(*doncloud, pc2_output);
    pc2_output.header.frame_id = "velodyne";
    pc2_output.header.stamp    = ros::Time::now();
    pub.publish(pc2_output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "doc_segmentation");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/rm_ground", 10, pcCallback);
    pub                 = n.advertise<sensor_msgs::PointCloud2>("/velodyne/don_segmentation", 1);
   
    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

   return 0;
}
