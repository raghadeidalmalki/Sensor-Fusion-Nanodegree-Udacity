#include <iostream>
#include <chrono>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

template<typename PointT>
class ProcessPointClouds {
public:
    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
};

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid point reduction
    typename pcl::PointCloud<PointT>::Ptr cloudVGFiltered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxelGridFilter;
    voxelGridFilter.setInputCloud(cloud);
    voxelGridFilter.setLeafSize(filterRes, filterRes, filterRes);
    voxelGridFilter.filter(*cloudVGFiltered);

    // Region of Interest (ROI) based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudBoxFiltered(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cropBoxFilter(true);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.setInputCloud(cloudVGFiltered);
    cropBoxFilter.filter(*cloudBoxFiltered);

    // Get the indices of rooftop points
    std::vector<int> indices;
    pcl::CropBox<PointT> roofFilter(true);
    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roofFilter.setInputCloud(cloudBoxFiltered);
    roofFilter.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point : indices)
        inliers->indices.push_back(point);

    // Remove the rooftop indices
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.setInputCloud(cloudBoxFiltered);
    extract.filter(*cloudFiltered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudFiltered;
}

int main(int argc, char** argv)
{
    // Check for correct number of arguments
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <output.pcd> <filterRes> <minPoint> <maxPoint>" << std::endl;
        return -1;
    }

    // Load point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        std::cerr << "Couldn't read file " << argv[1] << std::endl;
        return -1;
    }

    // Set filter resolution and bounds for crop box from command line arguments
    float filterRes = std::stof(argv[3]);
    Eigen::Vector4f minPoint(std::stof(argv[4]), std::stof(argv[5]), -1.0f, 1.0f);
    Eigen::Vector4f maxPoint(1.0f, 1.0f, 1.0f, 1.0f);

    // Process the point cloud
    ProcessPointClouds<pcl::PointXYZ> processor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = processor.FilterCloud(cloud, filterRes, minPoint, maxPoint);

    // Save the filtered point cloud
    pcl::io::savePCDFileASCII(argv[2], *filteredCloud);
    std::cout << "Saved filtered point cloud to " << argv[2] << std::endl;

    return 0;
}
