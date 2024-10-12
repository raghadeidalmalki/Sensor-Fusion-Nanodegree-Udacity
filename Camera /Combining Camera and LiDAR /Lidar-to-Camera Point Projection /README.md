# Lidar-to-Camera Point Projection 
The initial step in the fusion process involves combining the tracked feature points from the camera images with the 3D Lidar points. To achieve this, we need to geometrically project the Lidar points into the camera's coordinate system, allowing us to determine the position of each 3D Lidar point on the image sensor.
## Steps for Lidar-to-Camera Point Projection 
1.	Convert lidar points into Homogenous coordinates (to make transfomations easily; instead of solving lengthy equations, we can simply concatenate a number of vector-matrix-multiplications to complete the point projection efficiently).
2.	Map all points onto the image plane
3.	Move back to the Euclidean coordinate system with x and y to the position of where the lidar point hits the image

**Homogenous coordinates:**

<img width="400" alt="image" src="https://github.com/user-attachments/assets/59873edd-c50f-4372-b511-ea83b25d1c5e">

Transitioning from Lidar to camera coordinates requires applying translation and rotation operations to each 3D point. By using a linear transformation, 3D points can be represented as vectors, while operations like translation, rotation, scaling, and perspective projection can be expressed as matrices that multiply these vectors.

The challenge with the current projection equations is that they involve division by Z, introducing non-linearity that complicates transformation into a more manageable matrix-vector form. To overcome this, we can switch to a coordinate system known as the homogeneous coordinate system. Although moving between the original Euclidean system and the homogeneous system is a non-linear operation, once in homogeneous coordinates, projective transformations become linear and can be represented as straightforward matrix-vector multiplications.

Transformations between both coordinate systems work as shown in the following figure

<img width="400" alt="image" src="https://github.com/user-attachments/assets/49fc99cd-76a9-46b7-baee-2b926c3b2850">

A point in the n-dimensional Euclidean coordinate system is represented by a vector with n components. The transformation into (n+1)-dimensional homogeneous coordinates can be achieved by simply adding the number 1 as an additional component. The transformation can be applied to both image coordinates as well as scene coordinates.
A good summary of the deal between conversion from 3D to 2D coordinates is available [here](https://www.cse.psu.edu/~rtc12/CSE486/lecture12.pdf).

**Translation:** As seen in the following figure, translation describes the linear shift of a point **P** to a new location **P′** by adding the components of a translation vector **t** to the components of **P**.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/00ea6c10-d882-4f52-8ebb-4aeea64c1262">

In homogeneous coordinates, this can be expressed by concatenating an identity matrix I of size N and the translation vector **t**. Note that N represents the number of components in **P**. The translation operation then becomes a simple matrix-vector multiplication as shown in the figure above.

**Scale:** While translation involves adding a translation vector **t** to the components of **P**, scaling works by multiplying the components with a scale vector **s** instead. In homogeneous coordinates, this can be expressed as a matrix-vector multiplication as seen in the figure below.

![image](https://github.com/user-attachments/assets/bb5f09db-0817-4810-818c-210cd833c0b8)

Rotation : A point **P′** is rotated in counter-clockwise direction by using the following equations for x and y.

![image](https://github.com/user-attachments/assets/37fe6d4a-472c-4402-8a5b-b22076c3ad77)

As with previous operations, this can be expressed as a matrix-vector multiplication, with R representing the rotation matrix. In 3D space, a point P can be rotated around all three axes using the following rotation matrices:

![image](https://github.com/user-attachments/assets/0da8136e-c446-4b9e-9263-20da8a659a63)

The combined matrix comprising the rotation matrix R and the translation vector **t** is known as the extrinsic matrix. This matrix models how points are transformed between different coordinate systems. Once a point in the Lidar coordinate system is expressed in camera coordinates, the next step is to project it onto the image plane.

To accomplish this, we also need to incorporate the intrinsic parameters previously discussed. Using homogeneous coordinates, we can achieve this by concatenating the individual matrices as follows:

![image](https://github.com/user-attachments/assets/5544eec9-bf24-4e3f-aae2-67eb900f5cc9)


