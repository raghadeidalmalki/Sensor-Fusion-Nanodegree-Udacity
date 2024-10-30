# Udacity Sensor Fusion Nanodegree Program
The Sensor Fusion Engineer Nanodegree Program covers the essential skills to thrive as a sensor fusion engineer, with a particular emphasis on autonomous vehicles. We’ll delve into the integration of lidar, camera, and radar sensors—three critical components of automotive perception. Beyond the automotive sector, the expertise gained from this program is highly transferable, and applicable in a wide variety of domains.

# Lidar, Radar and Camera 
Lidar, radar, and cameras each come with their own advantages and disadvantages. They complement one another, which is why we fuse them together. Below are some of their key pros and cons.

![image](https://github.com/user-attachments/assets/bc32824a-a314-4920-b465-2f59af01f45e)
![image](https://github.com/user-attachments/assets/2c7a7820-d2c6-45a1-95e4-71288299b99a)

# Lidar Obstacle Detection Project
In this project, I analyzed several point cloud data files from a Lidar sensor to identify cars and other obstacles on a city street. The detection pipeline utilized Voxel Grid and ROI-based filtering, 3D RANSAC segmentation, Euclidean clustering using KD-Tree, and bounding box creation.

The final result is shown bellow:
![project1](https://github.com/user-attachments/assets/a4e08262-23a6-4fea-bed5-3ee2056fda7c)

## 3D Object Tracking based on Camera Project

This project focuses on tracking a vehicle ahead in the same lane and calculating the time-to-collision (TTC) using both camera images and Lidar data. To establish the TTC estimation process from camera feeds, I implemented techniques for keypoint detection, descriptor extraction, and matching keypoints between consecutive frames. By utilizing 3D bounding boxes, I extracted keypoints associated with the leading vehicle and determined the TTC based on the relative distances of matched keypoints in two successive images. Additionally, the matched keypoints aided in aligning 3D bounding boxes in the Lidar point cloud, allowing the Lidar system to estimate TTC by analyzing the nearest distances of these boxes to the ego vehicle across two consecutive frames.

![ttc lidar 9](https://github.com/user-attachments/assets/4dadfb97-1d33-4265-923b-51fb03553ad2)


## Velocity and Range Detection based on Radar Project

In this project, we begin by establishing a target with specific velocity and position, along with the relevant Radar specifications. The Radar wave signal is then propagated using the Frequency Modulated Continuous Wave (FMCW) model. To identify the target's range and velocity, we apply a 2D FFT to the received signal. Finally, a 2D Constant False Alarm Rate (CFAR) detector is utilized on the 2D FFT results to detect the target.

<img width="436" alt="358868287-e06ab0f8-3a57-4ac4-8201-f770544712b0" src="https://github.com/user-attachments/assets/04f48a16-3d31-41dd-a8a9-154d536c3bbf">

## Unscented Kalman Filter (fusing Lidar and Radar data) Project

In the final project, each moving vehicle, except for the ego vehicle, is assigned an unscented Kalman Filter (UKF). Lidar and Radar data for these vehicles are continuously input into their respective UKFs. This data is processed within the UKF framework, allowing for the estimation of each car's position and velocity through predict-update cycles.

The result below depicts a highway scenario. The ego vehicle is represented in green, while other cars are shown in blue. The red spheres above the vehicles indicate Lidar detections, and the purple lines illustrate Radar measurements, including velocity magnitude along the detected angle. The green spheres represent the predicted paths of the cars as estimated by the UKF for the near future.

![ukf](https://github.com/user-attachments/assets/ff6a2c75-e539-4d3c-98b4-92cf3fe60caf)

