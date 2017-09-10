#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>
#include <vector>
#include <algorithm>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>

#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/pcl_plotter.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/vfh.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/search/kdtree.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_conversions/pcl_conversions.h>

#define USETIME 0
// #define OUTPUT 0

#if (USETIME)
  #include <time.h>
#endif

#define RADIUS 0.05f // 0.03
#define K 10 // 10
#define TOP 10 // 10
#define DOWN_SAMPLE_SIZE 0.01f // 0.01

#if (USETIME)
  struct timeval start, end;
  long mtime, seconds, useconds;
  long depth_cut, downsampling, color_filtering, normal_calculation, vfh_calcualtion, total;
#endif

ros::Publisher filtered_pub;
ros::Publisher downsampled_pub;
ros::Publisher feature_publisher;

pcl::visualization::PCLPlotter * plotter;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  #if (USETIME)
    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    total = (depth_cut + downsampling + color_filtering + normal_calculation + vfh_calcualtion + mtime) / 100;
    std::cout << "dc:\t" << depth_cut * 1.0f / total << "%" << std::endl;
    std::cout << "ds:\t" << downsampling * 1.0f / total << "%" << std::endl;
    std::cout << "cf:\t" << color_filtering * 1.0f / total << "%" << std::endl;
    std::cout << "nor:\t" << normal_calculation * 1.0f / total << "%" << std::endl;
    std::cout << "vfh:\t" << vfh_calcualtion * 1.0f / total << "%" << std::endl;
    std::cout << "total:\t" << total * 100 << std::endl << std::endl;
    gettimeofday(&start, NULL);
  #endif

  // Init variables
  pcl::ExtractIndices<pcl::PointXYZRGB> eifilter (true); // Initializing with true will allow us to extract the removed indices
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud <pcl::PointXYZRGB>);

  geometry_msgs::Pose principal_direction;

  // Read the cloud from the ROS msg
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Perform the down-sampling

  // std::cout << cloud->size() << std::endl;

  pcl::VoxelGrid<pcl::PointXYZRGB> ds;
  ds.setInputCloud (cloud);
  ds.setLeafSize (DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE);
  ds.filter (*cloud_downsampled);

  // std::cout << cloud_filtered->size() << std::endl;

  #if (USETIME)
    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    downsampling = mtime;
    gettimeofday(&start, NULL);
  #endif

  // Cut the depth of the cloud
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_downsampled);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);

  pass.filter (*cloud_filtered);

  #if (USETIME)
    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    depth_cut = mtime;
    gettimeofday(&start, NULL);
  #endif


  // Extract the points with the aprropriate color
  pcl::PointXYZHSV temp;
  for(int i=0;i<cloud_filtered->size();i++)
  {
		pcl::PointXYZRGBtoXYZHSV(cloud_filtered->points[i],temp);
		if(temp.h>160 && temp.h<210 && temp.s>0.6)
    {
			inliers->indices.push_back(i);
      cloud_filtered->points[i].g = 255;
		}
	}

  eifilter.setInputCloud (cloud_filtered);
	eifilter.setIndices (inliers);
	eifilter.filter (*cloud_filtered);

  #if (USETIME)
    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    color_filtering = mtime;
    gettimeofday(&start, NULL);
  #endif

// Eliminate the outlier if the cloud is large enough
  if (cloud_filtered->size() > 0)
  {
    //for normal computing
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered);
    // ne.setSearchSurface (cloud_filtered);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    ne.setKSearch (K);
    // ne.setRadiusSearch (RADIUS);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);
    ne.compute (*cloud_with_normals);

    #if (USETIME)
      gettimeofday(&end, NULL);
      seconds  = end.tv_sec  - start.tv_sec;
      useconds = end.tv_usec - start.tv_usec;
      mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
      normal_calculation = mtime;
      gettimeofday(&start, NULL);
    #endif

    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud_filtered);
    vfh.setInputNormals (cloud_with_normals);
    vfh.setSearchMethod (tree);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
    vfh.compute (*vfhs);

    std_msgs::Float64MultiArray features;

    for (int i = 0; i < 135; i++)
      features.data.push_back(double(vfhs->points[0].histogram[i]));

    plotter->clearPlots ();
    plotter->addFeatureHistogram (*vfhs, 135, "cloud", 1440, 900);
    plotter->spinOnce (100);

    feature_publisher.publish (features);

    #if (USETIME)
      gettimeofday(&end, NULL);
      seconds  = end.tv_sec  - start.tv_sec;
      useconds = end.tv_usec - start.tv_usec;
      mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
      vfh_calcualtion = mtime;
      gettimeofday(&start, NULL);
    #endif
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 filtered_output;
  sensor_msgs::PointCloud2 downsampled_output;
  pcl::toROSMsg(*cloud_filtered, filtered_output);
  pcl::toROSMsg(*cloud_downsampled, downsampled_output);
  filtered_output.header = cloud_msg->header;
  downsampled_output.header = cloud_msg->header;

  // Publish the data
  filtered_pub.publish (filtered_output);
  downsampled_pub.publish (downsampled_output);
}

int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  plotter = new pcl::visualization::PCLPlotter ("My Plotter");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points_SR300_611205001943", 1, cloud_cb);
  feature_publisher = nh.advertise<std_msgs::Float64MultiArray> ("/soft_object_tracking/centroid", 2);

  // Create a ROS publisher for the output point cloud
  filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_output", 1);
  downsampled_pub = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_output", 1);

  #if (USETIME)
    gettimeofday(&start, NULL);
  #endif
  // Spin
  ros::spin ();

  return 0;
}
