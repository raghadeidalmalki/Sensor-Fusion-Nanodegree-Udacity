# Engineering a Collision Detection System
A Collision Avoidance System (CAS) is an active safety feature designed to alert drivers or automatically apply the brakes in the event of a potential collision with an object in the vehicle's path. When a vehicle is detected ahead, the CAS continuously calculates the time-to-collision (TTC). If the TTC drops below a predefined threshold, the system may either warn the driver of the imminent risk or, depending on its design, engage the brakes autonomously to prevent the collision.

Before we can calculate the time-to-collision (TTC), we need to develop a mathematical model to describe the relative motion between the vehicles.

All velocities referenced in the models below represent the relative velocities between the vehicle equipped with the sensor and the preceding vehicle being monitored by that sensor.

## Constant Velocity Model (CVM) 
To compute the TTC, we need to make assumptions on the physical behavior of the preceding vehicle. One assumption could be that the relative velocity between the yellow and green vehicle in the above figure were constant. This would lead to the so-called constant velocity model (CVM) which is represented by eq. 1 in the following diagram.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/1ae7b746-746e-4b8d-a997-1a80d42d9e0d">

Types of motion models possible:
1.	Constant velocity model (CVM) - The one we will be working on.
2.	Constant acceleration model (CAM) - An ideal case, but still complex as compared to the constant velocity motion model
3.	Changing acceleration - Real-life scenarios, most often too complex to handle in practice

Based on the constant-velocity model, the velocity v0 can be computed from two successive Lidar measurements as follows: 

<img width="400" alt="image" src="https://github.com/user-attachments/assets/6f88d5ba-7ace-4b03-9393-258f07b78f5a">

Once the relative velocity v0 is determined, the time to collision (TTC) can be calculated by dividing the remaining distance between the two vehicles by v0. With a Lidar sensor capable of precise distance measurements, a TTC estimation system can be developed using a Constant Velocity Model (CVM) along with the equations outlined above. However, it's important to note that a radar sensor would be a more effective solution for TTC calculation, as it can directly measure the relative speed. In contrast, using a Lidar sensor requires computing v0 from two (and potentially noisy) distance measurements.

## Estimating TTC with a LiDAR
To derive a stable TTC measurement from the given point cloud, two main steps have to be performed:
1.	Remove measurements on the road surface
2.	Remove measurements with low reflectivity

<img width="270" alt="image" src="https://github.com/user-attachments/assets/6362f437-40ea-4f12-8538-e3e70fc69beb">

<img width="400" alt="image" src="https://github.com/user-attachments/assets/72844ec6-470a-4b42-b251-8e0e0fad1de9">

## Computing TTC from Distance Measurements
In the code examples throughout this course, Lidar points are organized into a data structure called ```LidarPoints```. This structure includes the point coordinates: x (forward), y (left), and z (upward) in metric units, along with the point reflectivity r, which ranges from 0 to 1, indicating the level of reflectivity (with higher values representing greater reflectivity).

```ruby
struct LidarPoint { // single lidar point in space
    double x, y, z; // point position in m
    double r; // point reflectivity in the range 0-1
};
```
To calculate the time-to-collision (TTC), we first need to determine the distance to the closest Lidar point in the vehicle's path. In the figure below, Lidar measurements taken from the tailgate of the preceding vehicle at times t0 (green) and t1 (red) illustrate this. It is evident that the distance to the vehicle has decreased slightly between these two time points.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/cd33e842-b162-4bed-a4bd-87ca6bbcc77f">

The following code searches for the closest point in the point cloud associated with t0 (lidarPointsPrev) and in the point cloud associated with t1 (lidarPointsCurr). After finding the distance to the closest points respectively, the TTC is computed based on the formula we derived at the beginning of this section.

```ruby
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev, 
                     std::vector<LidarPoint> &lidarPointsCurr, double &TTC)
{
    // auxiliary variables
    double dT = 0.1; // time between two measurements in seconds

    // find closest distance to Lidar points 
    double minXPrev = 1e9, minXCurr = 1e9; //1e9 is 1*10^9
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
        minXPrev = minXPrev>it->x ? it->x : minXPrev;
    }

    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
        minXCurr = minXCurr>it->x ? it->x : minXCurr;
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev-minXCurr); // minXCurr * dT calculates the distance the object will travel in the next time interval (dT) based on its current velocity (assuming it's moving at a constant rate).
}
```

**Loop through lidarPointsPrev:**
This loop iterates through each LidarPoint in lidarPointsPrev. For each point, it checks if it->x (the x-coordinate of the current point) is smaller than minXPrev. If it is, minXPrev is updated to it->x.
```
for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
    minXPrev = minXPrev > it->x ? it->x : minXPrev;
}
```
**Loop through lidarPointsCurr:**
This loop iterates through each LidarPoint in lidarPointsCurr and updates minXCurr to be the smallest x-coordinate found among these points.
```
for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
    minXCurr = minXCurr > it->x ? it->x : minXCurr;
}
```
**Compute Time-to-Collision (TTC):**
After finding the minimum x-coordinates (minXPrev and minXCurr) for the previous and current Lidar points respectively, the TTC is computed using the formula: 
```
TTC = minXCurr * dT / (minXPrev - minXCurr);
```
## Estimating TTC with a Camera
Computing the time-to-collision (TTC) with a 2D camera is more complex. First, a camera captures only a 2D image of the scene, lacking the ability to perform 3D measurements. Second, we must reliably and accurately identify vehicles to track their motion over time. Monocular cameras cannot measure metric distances; they are passive sensors that depend on ambient light reflecting off objects into the camera lens. As a result, it is not possible to measure the travel time of light as is done with Lidar technology.

Despite the limitations of a monocular camera, we can explore a method to compute time-to-collision (TTC) without directly measuring distances. By considering the constant velocity motion model, we can replace the metric distances d with a more reliable measure: pixel distances on the image plane.
In the figure below, you can see how the height H of the preceding vehicle is projected onto the image plane through perspective projection. This mapping shows that the same height H corresponds to different heights h0 and h1 on the image plane, depending on the distances d0 and d1 of the vehicle. This highlights the geometric relationship between h, H, d, and the focal length f of the pinhole camera—an aspect we aim to leverage in the following discussion.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/533d7766-fd7a-461a-bd4c-cec74f174f27">
<img width="400" alt="image" src="https://github.com/user-attachments/assets/0183a9a3-55ac-4500-9a52-fe430a5e6b02">

•	In (1) we use the focal length of the camera and a distance measurement d0 performed at time t0 to project the height H of the vehicle onto the image plane and thus to a height h0 in pixels. The same is done at time t1, leading to a projected height h1.

•	In (2), we compute the ratio of the relative heights h0 and h1. As both H and f are cancelled out, we can observe a direct relation between relative height h and absolute metric distance d. We can thus express the distance to the vehicle d0 as the product of d1 and the ratio of relative heights on the image plane.

•	In (3), we substitute d0 in the equation for constant velocity and solve for d1, which is now dependent on the constant relative velocity v0, on the time between measuring d0 and d1 and on the ratio of relative heights on the image plane.

•	In (4), the TTC is computed as the ratio of remaining distance to impact, which is d1, and the constant velocity v0. As we can easily see, the TTC now only consists of Δt, h0 and h1.

### Computing TTC from Relative Keypoint Distances
Instead of depending on the detection of the entire vehicle, we aim to analyze its structure on a finer scale. If we could identify unique keypoints that can be reliably tracked from one frame to the next, we could utilize the distances between these keypoints on the vehicle relative to one another. This would allow us to compute a robust estimate of the height ratio for our TTC equation. The following figure illustrates this concept.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/28a831de-66d9-4780-b4b7-53c2161e6380">

The ratio of all relative distances between each other can be used to compute a reliable TTC estimate by replacing the height ratio h1/h0 with the mean or median of all distance ratios dk/dk′

In the code examples in this course, matching keypoints between images are packaged into an OpenCV data structure called ```cv::DMatch```. To retrieve the keypoints associated with a match both in the current frame and in the previous frame, we need to make use of the attributes ```queryIdx``` and ```trainIdx```, which you will find in the example code below. For more information on the ```DMatch``` data structure and its attributes, refer to the [OpenCV documentation](https://docs.opencv.org/4.4.0/d4/de0/classcv_1_1DMatch.html).

***All matched keypoints are stored in a dynamic list, which is then passed to a function called ``computeTTCCamera``, which returns the time-to-collision for each object in the scene.***

## TTC Building Blocks
### Camera

**Main idea:** compute keypoint matches between successive images and observe he relative distances between them  

**Problem:** we need to isolate keypoints on the preceding vehicle i.e. exclude the road surface and static objects 


<img width="200" alt="image" src="https://github.com/user-attachments/assets/7f8f74d8-7f73-4e24-af54-5457dbe913e5">

### Lidar
**Main idea:** compute distance to preceding vehicle from 3d point cloud at successive moments in time 

**Problem:** we need to isolate 3d points on the preceding vehicles, i.e. exclude the road surface and static objects.

Cropping will not be reliable enough, especially when the preceding vehicle is not directly in front of the sensor.

<img width="200" alt="image" src="https://github.com/user-attachments/assets/6df534bc-b06d-45f3-b468-f28e1fe9309a">

### Fusing camera and lidar 
**Solution:** use camera-based object classification to cluster lidar points and compute a TTC estimate from 3D bounding boxes.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/43ab0b44-136b-484c-940d-d9c8fed1c307">
