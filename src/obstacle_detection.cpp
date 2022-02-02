#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include "std_msgs/Bool.h" 

ros::Publisher pub;
ros::Publisher object;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passfilteredx(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passfilteredy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier (new pcl::PointCloud<pcl::PointXYZ>);
    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Outlier filtering (removes stray points)
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sori;
    sori.setInputCloud (cloud);
    sori.setMeanK (50);
    sori.setStddevMulThresh (3);
    sori.filter (*cloud_outlier);
    
    
    //Passthrough filtering (set maximum and minimum bounds)
    pcl::PassThrough<pcl::PointXYZ> passx;
    passx.setInputCloud (cloud_outlier);
    passx.setFilterFieldName ("x");
    passx.setFilterLimits (0.5, 1.2);
    passx.filter (*cloud_passfilteredx);
    
    //Passthrough filtering (set maximum and minimum bounds)
    pcl::PassThrough<pcl::PointXYZ> passy;
    passy.setInputCloud (cloud_passfilteredx);
    passy.setFilterFieldName ("y");
    passy.setFilterLimits (-0.4, 0.4);
    passy.filter (*cloud_passfilteredy);
    

    
    // Voxelgrid filtering (reduces number of points)
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud_passfilteredy);
    vox.setLeafSize (0.01, 0.01, 0.01);
    vox.filter (*cloud_filtered);
    
    
    //Segmentation step

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.04);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients); 
    
    //Extraction step

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*cloud_segmented);

    //Boolean variable to signify if any obstacles were detected
    std_msgs::Bool objectDetected;
    objectDetected.data=false;


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_segmented);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_segmented);
    ec.extract (cluster_indices);
    ROS_INFO_STREAM(cluster_indices.size()<<" clusters extracted ");
    if(cluster_indices.size()>0)
    {
        objectDetected.data=true;
    }

    //Publishing detection of objects 
    object.publish(objectDetected);
    visualization_msgs::MarkerArray markerArray;


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_segmented)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        //cloud_cluster->is_dense = true;
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_segmented, output);
    output.header.frame_id = "ifm3d/camera_link";

    // Publish the data
    pub.publish (output);
    
}
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "agv_vision");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("ifm3d/camera/cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  //ROS Publisher tht publishes TRUE if obstacle detected, else FALSE
  object= nh.advertise<std_msgs::Bool> ("object", 1);//To be changed

  // Spin
  ros::spin ();
}
