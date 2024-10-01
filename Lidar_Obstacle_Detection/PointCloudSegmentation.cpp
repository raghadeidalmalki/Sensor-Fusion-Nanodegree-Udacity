#include <iostream> 
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int main() {
  // Define pointers to hold point cloud data
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Read point cloud data from file
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);

  // Output the number of points in the original point cloud
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create a voxel grid filter object and downsample the point cloud
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);  // Set the voxel size
  sor.filter (*cloud_filtered_blob);  // Apply the voxel grid filter

  // Convert the PCLPointCloud2 blob to PointXYZ point cloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  // Output the number of points in the filtered point cloud
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled point cloud to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  // Define objects to hold model coefficients and inlier indices
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object for plane segmentation using RANSAC
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create object for extracting indices
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Initialize variables for the loop
  int i = 0, nr_points = (int) cloud_filtered->size ();

  // Perform segmentation and extraction until less than 30% of points remain
  while (cloud_filtered->size () > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);

    // Check if any inliers were found; if not, terminate the loop
    if (inliers->indices.size () == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);  // Extract the plane
    extract.filter (*cloud_p);  // Store the extracted plane points

    // Output the number of points in the extracted plane
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Write the extracted plane to disk
    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Extract the non-plane points (negatives) and update the filtered cloud
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);  // Swap to update cloud_filtered with non-plane points
    i++;
  }

  // Return 0 to indicate successful completion of the program
  return (0);
}
