# LiDAR Obstacle Detection
Lidar works by emitting brief laser pulses that travel to an object and back, measuring the time taken for the round trip to produce high-resolution data through the analysis of thousands of these signals.

## Point Cloud 
![1679066814413](https://github.com/user-attachments/assets/4132aa55-3be7-4a98-8339-b9be7bcac718)


A **Point Cloud** is a collection of all LiDAR reflections, where each point corresponds to a specific laser beam reflection (the laser beam has a narrow diameter).

**Point Cloud Data (PCD):** Lidar data is stored in the PCD format. A pcd file contains a list of (x, y, z) cartesian coordinates along with their associated intensity values.


**Point Cloud Library (PCL):** PCL is an open-source C++ library designed for working with point clouds, it offers a variety of built-in processing functions. We will utilize it to visualize data and render shapes. You can find additional documentation for PCL [here](https://pointclouds.org/).


## Point Cloud Segmentation

![image](https://github.com/user-attachments/assets/4d3bec64-b127-47da-897e-0620a21500a3)

With **segmentation** we are associating certain point (from the point cloud data) with objects, for this we will be using a method called **Planer Segmentation** which uses the **RANSAC Algorithm**

### Random Sample Consensus (RANSAC)

 A robust algorithm designed to identify a mathematical model in datasets that may contain a significant number of outliers. It iteratively selects random subsets of the data to estimate the model, refining its parameters based on consensus from the larger dataset, ultimately improving accuracy and reliability in the presence of noise.

 ![image](https://github.com/user-attachments/assets/00b06593-1783-4bec-b39b-1d8bf1478ca9)

- Iterative method 

-	Each iteration randomly picks subset of points 

-	Fit a model to the points

-	The iteration with the most inliers to the model will be chosen

##  Clustering Obstacles

Through **clustering** we are grouping the segmented point cloud into objects for easier tracking. One effective method for grouping and clustering point cloud data is known as **Euclidean clustering**. This technique organizes points based on their spatial proximity, enabling the identification of distinct clusters within the dataset.

### Euclidean Clustering
Euclidean clustering involves grouping points based on their proximity to one another. To efficiently perform nearest neighbor searches, a KD-Tree data structure is employed. This structure significantly enhances lookup speed from O(n) to O(log(n)), as it optimally partitions the search space. By organizing points into regions within the KD-Tree, you can effectively eliminate the need to calculate distances for potentially thousands of points that are clearly too far away to be relevant, streamlining the clustering process.


![image](https://github.com/user-attachments/assets/085ede81-7317-42d6-9b5c-897add694f3e)

