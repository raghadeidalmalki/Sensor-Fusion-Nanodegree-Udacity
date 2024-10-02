#include <iostream>
#include <unordered_set>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Function to perform RANSAC for plane fitting
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(static_cast<unsigned>(time(NULL))); // Seed for random number generation

    while (maxIterations--) {
        std::unordered_set<int> inliers;
        // Randomly sample three points
        while (inliers.size() < 3)
            inliers.insert(rand() % (cloud->points.size()));

        auto it = inliers.begin();
        pcl::PointXYZ point1 = cloud->points[*it];
        it++;
        pcl::PointXYZ point2 = cloud->points[*it];
        it++;
        pcl::PointXYZ point3 = cloud->points[*it];

        // Vectors v1 and v2
        float v1_x = point2.x - point1.x;
        float v1_y = point2.y - point1.y;
        float v1_z = point2.z - point1.z;

        float v2_x = point3.x - point1.x;
        float v2_y = point3.y - point1.y;
        float v2_z = point3.z - point1.z;

        // Cross product v1 × v2 to get the normal vector (A, B, C)
        float A = v1_y * v2_z - v1_z * v2_y;
        float B = v1_z * v2_x - v1_x * v2_z;
        float C = v1_x * v2_y - v1_y * v2_x;
        float D = -(A * point1.x + B * point1.y + C * point1.z);

        // Measure distance from each point to the plane
        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0)
                continue;

            pcl::PointXYZ point = cloud->points[index];
            float x = point.x;
            float y = point.y;
            float z = point.z;

            // Distance from the point to the plane
            float distance = fabs(A * x + B * y + C * z + D) / sqrt(A * A + B * B + C * C);

            if (distance <= distanceTol)
                inliers.insert(index); // Add index to inliers if within distance
        }

        // Update the best inliers set if this one is better
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

    return inliersResult;
}

int main() {
    // Create a sample point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 100; // Number of points
    cloud->height = 1;  // Single row
    cloud->points.resize(cloud->width * cloud->height);

    // Fill in the point cloud with random points simulating a plane
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = static_cast<float>(rand() % 100); // Random x-coordinate
        cloud->points[i].y = static_cast<float>(rand() % 100); // Random y-coordinate
        cloud->points[i].z = 0.5f * cloud->points[i].x + 0.2f * cloud->points[i].y + static_cast<float>(rand() % 5); // Simulate a plane with some noise
    }

    // Call the RansacPlane function
    int maxIterations = 100; // Set number of iterations
    float distanceTol = 1.0; // Distance threshold for considering inliers
    std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceTol);

    // Output the indices of inliers
    std::cout << "Inliers Indices: ";
    for (int index : inliers) {
        std::cout << index << " ";
    }
    std::cout << std::endl;

    return 0;
}
