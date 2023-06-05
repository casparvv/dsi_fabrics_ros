// LiDAR point cloud processing node.

// To do: converse data
// Current DSI fabrics input (n*2,) --processing in planner--> (n, 3)
// Give pcl output to adapt_runtime_arguments as third argument
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // // Callback with downsampled VoxelGrid filter
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "lidar_processor");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/lidar/pointcloud", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud_filtered", 1);

  ros::spin ();
}

