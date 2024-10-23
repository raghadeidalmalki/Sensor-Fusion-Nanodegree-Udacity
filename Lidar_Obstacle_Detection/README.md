# LiDAR Obstacle Detection
Lidar works by emitting brief laser pulses that travel to an object and back, measuring the time taken for the round trip to produce high-resolution data through the analysis of thousands of these signals.

## Point Cloud 
![1679066814413](https://github.com/user-attachments/assets/4132aa55-3be7-4a98-8339-b9be7bcac718)


**Point Cloud:** A collection of all LiDAR reflections, where each point corresponds to a specific laser beam reflection (the laser beam has a narrow diameter).

**Point Cloud Data (PCD):** Lidar data is stored in the PCD format. A pcd file contains a list of (x, y, z) cartesian coordinates along with their associated intensity values.


**Point Cloud Library (PCL):** PCL is an open-source C++ library designed for working with point clouds, it offers a variety of built-in processing functions. We will utilize it to visualize data and render shapes. You can find additional documentation for PCL [here](https://pointclouds.org/).





