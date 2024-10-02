#include <iostream>
#include <unordered_set>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Function to perform RANSAC for line fitting
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(static_cast<unsigned>(time(NULL))); // Seed for random number generation

    while (maxIterations--) {
        std::unordered_set<int> inliers; // To hold indices of inliers
        // Randomly sample two points
        while (inliers.size() < 2) {
            inliers.insert(rand() % (cloud->points.size()));
        }

        // Get the coordinates of the two sampled points
        float x1, y1, x2, y2;
        auto it = inliers.begin();
        x1 = cloud->points[*it].x;
        y1 = cloud->points[*it].y;
        it++;
        x2 = cloud->points[*it].x;
        y2 = cloud->points[*it].y;

        // Calculate line coefficients (Ax + By + C = 0)
        float A = y1 - y2;
        float B = x2 - x1;
        float C = x1 * y2 - x2 * y1;

        // Measure distance from all other points to the line
        for (int index = 0; index < cloud->points.size(); index++) {
            // Skip the sampled points
            if (inliers.count(index) > 0) continue;

            pcl::PointXYZ point = cloud->points[index];
            float x3 = point.x;
            float y3 = point.y;

            // Calculate the distance from the point to the line
            float d = fabs(A * x3 + B * y3 + C) / sqrt(A * A + B * B);
            if (d <= distanceTol) {
                inliers.insert(index); // Add index to inliers if within distance
            }
        }

        // Update the best inliers set if this one is better
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    return inliersResult;
}

int main() {
    // Create a sample point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 10;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // Fill in the point cloud with random points
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = rand() % 100; // Random x-coordinate
        cloud->points[i].y = rand() % 100; // Random y-coordinate
        cloud->points[i].z = 0;             // Set z-coordinate to 0 for 2D line fitting
    }

    // Call the Ransac function
    int maxIterations = 100;
    float distanceTol = 1.0; // Distance threshold for considering inliers
    std::unordered_set<int> inliers = Ransac(cloud, maxIterations, distanceTol);

    // Output the indices of inliers
    std::cout << "Inliers Indices: ";
    for (int index : inliers) {
        std::cout << index << " ";
    }
    std::cout << std::endl;

    return 0;
}
