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
// #include <pcl/visualization/cloud_viewer.h>

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
#define TARGET 1

#if (USETIME)
  #include <time.h>
#endif

#define RADIUS 0.015f // 0.03
#define K 10 // 10
#define TOP 10 // 10
#define DOWN_SAMPLE_SIZE 0.01f // 0.01

#if (USETIME)
  struct timeval start, end;
  long mtime, seconds, useconds;
  long depth_cut, downsampling, color_filtering, normal_calculation, curvature_calcualtion;
#endif

#if (TARGET)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB target_centroid;
  pcl::PointXYZRGB target_end_point;
  void process_target(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud);
#endif

ros::Publisher filtered_pub;
ros::Publisher downsampled_pub;
ros::Publisher feature_publisher;
ros::Publisher max_cur_pub;
// ros::Publisher avg_cur_pub;
ros::Publisher vector_pub;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

//return the centroid of given points
pcl::PointXYZRGB getCentroid(pcl::PointCloud <pcl::PointXYZRGB> p)
{
  pcl::PointXYZRGB centroid;
  for(int i=0;i<p.size();i++)
  {
    centroid.x += p[i].x;
    centroid.y += p[i].y;
    centroid.z += p[i].z;
  }
  centroid.x /= p.size();
  centroid.y /= p.size();
  centroid.z /= p.size();
  return centroid;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  #if (USETIME)
    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    std::cout << "dc:\t" << depth_cut * 1.0f / (depth_cut + downsampling + color_filtering + normal_calculation + curvature_calcualtion + mtime) * 100 << "%" << std::endl;
    std::cout << "ds:\t" << downsampling * 1.0f / (depth_cut + downsampling + color_filtering + normal_calculation + curvature_calcualtion + mtime) * 100 << "%" << std::endl;
    std::cout << "cf:\t" << color_filtering * 1.0f / (depth_cut + downsampling + color_filtering + normal_calculation + curvature_calcualtion + mtime) * 100 << "%" << std::endl;
    std::cout << "nor:\t" << normal_calculation * 1.0f / (depth_cut + downsampling + color_filtering + normal_calculation + curvature_calcualtion + mtime) * 100 << "%" << std::endl;
    std::cout << "cur:\t" << curvature_calcualtion * 1.0f / (depth_cut + downsampling + color_filtering + normal_calculation + curvature_calcualtion + mtime) * 100 << "%" << std::endl;
    std::cout << "total:\t" << depth_cut + downsampling + color_filtering + normal_calculation + curvature_calcualtion + mtime << std::endl << std::endl;
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

  // // Cut the depth of the cloud
  // pcl::PassThrough<pcl::PointXYZRGB> pass;
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.0, 2.0);
  //
  // pass.filter (*cloud_filtered);
  //
  // #if (USETIME)
  //   gettimeofday(&end, NULL);
  //   seconds  = end.tv_sec  - start.tv_sec;
  //   useconds = end.tv_usec - start.tv_usec;
  //   mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
  //   depth_cut = mtime;
  //   gettimeofday(&start, NULL);
  // #endif

  // // Perform the down-sampling
  // pcl::VoxelGrid<pcl::PointXYZRGB> ds;
  // ds.setInputCloud (cloud_filtered);
  // ds.setLeafSize (DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE);
  // ds.filter (*cloud_downsampled);
  //
  // #if (USETIME)
  //   gettimeofday(&end, NULL);
  //   seconds  = end.tv_sec  - start.tv_sec;
  //   useconds = end.tv_usec - start.tv_usec;
  //   mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
  //   downsampling = mtime;
  //   gettimeofday(&start, NULL);
  // #endif

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
  if (cloud_filtered->size() > 100)
  {
    //pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    //sor.setInputCloud (cloud_filtered);
    //sor.setMeanK (50);
    //sor.setStddevMulThresh (1.0);
    //sor.filter (*cloud_filtered);

    std_msgs::Float64MultiArray features;
    features.data.resize(7);

    // Calculate the curvature
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

      // Setup the principal curvatures computation
      pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

      // Provide the original point cloud (without normals)
      principal_curvatures_estimation.setInputCloud (cloud_filtered);

      // Provide the point cloud with normals
      principal_curvatures_estimation.setInputNormals (cloud_with_normals);

      // Use the same KdTree from the normal estimation
      principal_curvatures_estimation.setSearchMethod (tree);
      principal_curvatures_estimation.setRadiusSearch (1.0);

      // Actually compute the principal curvatures
      pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
      principal_curvatures_estimation.compute (*principal_curvatures);

      #if (USETIME)
        gettimeofday(&end, NULL);
        seconds  = end.tv_sec  - start.tv_sec;
        useconds = end.tv_usec - start.tv_usec;
        mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
        curvature_calcualtion = mtime;
        gettimeofday(&start, NULL);
      #endif

      //find the max curvature and show its principal direction
      // float sum_curvature = 0;
      float max_curvature = 0;
      int distance = 0;
      // std::vector<float> *valid_curvatures = new std::vector<float>();
      for(int i=0;i<principal_curvatures->size();i++)
      {
        float curvature = principal_curvatures->points[i].pc1;
        if (!std::isnan(curvature))
        {
          if (curvature > max_curvature)
          {
            max_curvature = curvature;
            distance = i;
          }
          // sum_curvature += curvature;
          // valid_curvatures->push_back(curvature) ;
        }
      }
      // int size = valid_curvatures->size();
      // float top_sum = 0;
      // if (size > 0)
      if (distance > 0)
      {
        // int distance = std::distance (valid_curvatures->begin(), std::max_element (valid_curvatures->begin(), valid_curvatures->end()));

        pcl::PrincipalCurvatures desired_curvature = (*principal_curvatures)[distance];
        // pcl::PointXYZRGB *start_point = &((*cloud_filtered)[distance]);
        pcl::PointXYZRGB centroid_3D = getCentroid(*cloud_filtered);
        pcl::PointXYZRGB end_point(centroid_3D);
        end_point.x += desired_curvature.principal_curvature_x/20;
        end_point.y += desired_curvature.principal_curvature_y/20;
        end_point.z += desired_curvature.principal_curvature_z/20;
        centroid_3D.r = 255;
        centroid_3D.b = 255;
        end_point.r = 255;
        end_point.b = 255;

        Eigen::Quaternionf rot;
        Eigen::Vector3f x_axis (1, 0, 0);
        Eigen::Vector3f direction_vector (
          desired_curvature.principal_curvature_x,
          desired_curvature.principal_curvature_y,
          desired_curvature.principal_curvature_z);
        rot.setFromTwoVectors	(x_axis, direction_vector);

        principal_direction.orientation.x = rot.x();
        principal_direction.orientation.y = rot.y();
        principal_direction.orientation.z = rot.z();
        principal_direction.orientation.w = rot.w();
        principal_direction.position.x = centroid_3D.x;
        principal_direction.position.y = centroid_3D.y;
        principal_direction.position.z = centroid_3D.z;


        features.data[0] = desired_curvature.principal_curvature_x;
        features.data[1] = desired_curvature.principal_curvature_y;
        features.data[2] = desired_curvature.principal_curvature_z;
        // features.data[3] = (cloud_filtered->begin()+distance)->x;
        // features.data[4] = (cloud_filtered->begin()+distance)->y;
        // features.data[5] = (cloud_filtered->begin()+distance)->z;
        features.data[3] = centroid_3D.x;
        features.data[4] = centroid_3D.y;
        features.data[5] = centroid_3D.z;
        features.data[6] = max_curvature;

        feature_publisher.publish (features);

        // std::sort (valid_curvatures->begin(), valid_curvatures->end());
        //
        std_msgs::Float32 max_cur;
        max_cur.data = max_curvature;
        max_cur_pub.publish (max_cur);
        // for (int i = size-1; i >= 0 && i >= size - TOP; i--)
        // {
        //   top_sum += (*valid_curvatures)[i];
        // }
        // std_msgs::Float32 avg_cur;
        // avg_cur.data = top_sum / (size > TOP ? TOP : size);
        // avg_cur_pub.publish (avg_cur);
        //

        // #if (OUTPUT)
        // {
        //   std::cout << "avg of " << (size > TOP ? TOP : size) << " max:\t" << top_sum / (size > TOP ? TOP : size) << std::endl;
        //   std::cout << "total:\t\t" << size << std::endl;
        //   std::cout << "max:\t\t" << max_cur.data << std::endl;
        //   std::cout << "avg:\t\t" << avg_cur.data << std::endl << std::endl;
        // }
        // #endif
        // pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr const_cloud(cloud_filtered);
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->setBackgroundColor (0, 0, 0);

        cloud_filtered->push_back(centroid_3D);
        cloud_filtered->push_back(end_point);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "sample cloud");
        viewer->addArrow<pcl::PointXYZRGB>(end_point, centroid_3D, 0.15, 1, 0.15, false);

        #if (TARGET)
          pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> target_rgb(target_cloud);
          viewer->addPointCloud<pcl::PointXYZRGB> (target_cloud, target_rgb, "target cloud");
          viewer->addArrow<pcl::PointXYZRGB>(target_end_point, target_centroid, 1, 0.2, 0.2, false, "target arrow");
        #endif

        // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        // viewer->addCoordinateSystem (1.0);
        // viewer->initCameraParameters ();
        viewer->spinOnce (50);

      }
      else
      {
        std::cout << "No valid points!" << std::endl << std::endl;
      }
      // delete valid_curvatures;
    }
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 filtered_output;
  sensor_msgs::PointCloud2 downsampled_output;
  geometry_msgs::PoseStamped principal_direction_output;
  pcl::toROSMsg(*cloud_filtered, filtered_output);
  pcl::toROSMsg(*cloud_downsampled, downsampled_output);
  principal_direction_output.pose = principal_direction;
  filtered_output.header = cloud_msg->header;
  downsampled_output.header = cloud_msg->header;
  principal_direction_output.header = cloud_msg->header;

  // Publish the data
  filtered_pub.publish (filtered_output);
  downsampled_pub.publish (downsampled_output);
  vector_pub.publish (principal_direction_output);

}

int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points_SR300_611205001943", 1, cloud_cb);
  feature_publisher = nh.advertise<std_msgs::Float64MultiArray> ("/soft_object_curvature/feature", 2);

  // Create a ROS publisher for the output point cloud
  filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_output", 1);
  downsampled_pub = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_output", 1);
  max_cur_pub = nh.advertise<std_msgs::Float32> ("max_cur", 1);
  // avg_cur_pub = nh.advertise<std_msgs::Float32> ("avg_cur", 1);
  vector_pub = nh.advertise<geometry_msgs::PoseStamped> ("vector", 1);

  #if (TARGET)
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (ros::package::getPath("soft_object_curvature")+"/data/target.pcd", *target_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file target.pcd \n");
      return (-1);
    }
    process_target(target_cloud);
  #endif

  #if (USETIME)
    gettimeofday(&start, NULL);
  #endif
  // Spin
  ros::spin ();

  return 0;
}

#if (TARGET)
  void process_target(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud)
  {
    // Init variables
    pcl::ExtractIndices<pcl::PointXYZRGB> eifilter (true); // Initializing with true will allow us to extract the removed indices
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::VoxelGrid<pcl::PointXYZRGB> ds;
    ds.setInputCloud (target_cloud);
    ds.setLeafSize (DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE);
    ds.filter (*target_cloud);

    // Cut the depth of the cloud
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (target_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);

    pass.filter (*target_cloud);
    // Extract the points with the aprropriate color
    pcl::PointXYZHSV temp;
    for(int i=0;i<target_cloud->size();i++)
    {
  		pcl::PointXYZRGBtoXYZHSV(target_cloud->points[i],temp);
  		if(temp.h>160 && temp.h<210 && temp.s>0.6)
      {
  			inliers->indices.push_back(i);
        target_cloud->points[i].r = 255;
        target_cloud->points[i].g = 70;
        target_cloud->points[i].b = 70;
  		}
  	}

    eifilter.setInputCloud (target_cloud);
  	eifilter.setIndices (inliers);
  	eifilter.filter (*target_cloud);

    // Eliminate the outlier if the cloud is large enough
    if (target_cloud->size() > 100)
    {
    // Calculate the curvature
      //for normal computing
      pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud (target_cloud);
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      ne.setSearchMethod (tree);
      ne.setKSearch (K);
      // ne.setRadiusSearch (RADIUS);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);
      ne.compute (*cloud_with_normals);

      pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;
      principal_curvatures_estimation.setInputCloud (target_cloud);
      principal_curvatures_estimation.setInputNormals (cloud_with_normals);
      principal_curvatures_estimation.setSearchMethod (tree);
      principal_curvatures_estimation.setRadiusSearch (1.0);

      // Actually compute the principal curvatures
      pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
      principal_curvatures_estimation.compute (*principal_curvatures);

      float max_curvature = 0;
      int distance = 0;
      for(int i=0;i<principal_curvatures->size();i++)
      {
        float curvature = principal_curvatures->points[i].pc1;
        if (!std::isnan(curvature))
        {
          if (curvature > max_curvature)
          {
            max_curvature = curvature;
            distance = i;
          }
        }
      }

      if (distance > 0)
      {
        pcl::PrincipalCurvatures desired_curvature = (*principal_curvatures)[distance];
        target_centroid = getCentroid(*target_cloud);
        target_end_point = pcl::PointXYZRGB(target_centroid);
        target_end_point.x += desired_curvature.principal_curvature_x/20;
        target_end_point.y += desired_curvature.principal_curvature_y/20;
        target_end_point.z += desired_curvature.principal_curvature_z/20;
        target_centroid.r = 255;
        target_centroid.g = 255;
        target_end_point.r = 255;
        target_end_point.g = 255;

        target_cloud->push_back(target_centroid);
        target_cloud->push_back(target_end_point);

      }
      else
      {
        std::cout << "Target Has No valid points!" << std::endl << std::endl;
      }
    }
  }
#endif
