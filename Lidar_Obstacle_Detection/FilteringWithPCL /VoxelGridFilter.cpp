//Downsampling a PointCloud using a VoxelGrid filter
// Include the iostream library for input and output operations.
#include <iostream>
// Include the PCL (Point Cloud Library) header for reading PCD files.
#include <pcl/io/pcd_io.h>
// Include the header for point types used in PCL.
#include <pcl/point_types.h>
// Include the header for the Voxel Grid filter in PCL.
#include <pcl/filters/voxel_grid.h>

// Main function where execution starts.
int main () {
  // Create a smart pointer for a PCL point cloud object.
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  // Create a smart pointer for a filtered PCL point cloud object.
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Instantiate a reader for PCD files.
  pcl::PCDReader reader;
  // Read the PCD file from the specified path and load it into the cloud object.
  reader.read ("table_scene_lms400.pcd", *cloud); // Make sure the file is downloaded!

  // Output the number of data points in the original cloud to the standard error stream.
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create a VoxelGrid filter object for downsampling the point cloud.
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // Set the input cloud for the filter.
  sor.setInputCloud (cloud);
  // Set the size of the voxel grid (leaf size).
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  // Apply the filter and store the result in the filtered cloud.
  sor.filter (*cloud_filtered);

  // Output the number of data points in the filtered cloud to the standard error stream.
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  // Instantiate a writer for PCD files.
  pcl::PCDWriter writer;
  // Write the filtered point cloud to a new PCD file.
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  // Return 0 to indicate successful execution of the program.
  return (0);
}
