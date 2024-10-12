# Engineering a Collision Detection System
A Collision Avoidance System (CAS) is an active safety feature designed to alert drivers or automatically apply the brakes in the event of a potential collision with an object in the vehicle's path. When a vehicle is detected ahead, the CAS continuously calculates the time-to-collision (TTC). If the TTC drops below a predefined threshold, the system may either warn the driver of the imminent risk or, depending on its design, engage the brakes autonomously to prevent the collision.

Before we can calculate the time-to-collision (TTC), we need to develop a mathematical model to describe the relative motion between the vehicles.

All velocities referenced in the models below represent the relative velocities between the vehicle equipped with the sensor and the preceding vehicle being monitored by that sensor.

## Constant Velocity Model (CVM) 
To compute the TTC, we need to make assumptions on the physical behavior of the preceding vehicle. One assumption could be that the relative velocity between the yellow and green vehicle in the above figure were constant. This would lead to the so-called constant velocity model (CVM) which is represented by eq. 1 in the following diagram.

![image](https://github.com/user-attachments/assets/1ae7b746-746e-4b8d-a997-1a80d42d9e0d)

Types of motion models possible:
1.	Constant velocity model (CVM) - The one we will be working on.
2.	Constant acceleration model (CAM) - An ideal case, but still complex as compared to the constant velocity motion model
3.	Changing acceleration - Real-life scenarios, most often too complex to handle in practice

Based on the constant-velocity model, the velocity v0 can be computed from two successive Lidar measurements as follows: 

![image](https://github.com/user-attachments/assets/6f88d5ba-7ace-4b03-9393-258f07b78f5a)

Once the relative velocity v0 is determined, the time to collision (TTC) can be calculated by dividing the remaining distance between the two vehicles by v0. With a Lidar sensor capable of precise distance measurements, a TTC estimation system can be developed using a Constant Velocity Model (CVM) along with the equations outlined above. However, it's important to note that a radar sensor would be a more effective solution for TTC calculation, as it can directly measure the relative speed. In contrast, using a Lidar sensor requires computing v0 from two (and potentially noisy) distance measurements.

To derive a stable TTC measurement from the given point cloud, two main steps have to be performed:
1.	Remove measurements on the road surface
2.	Remove measurements with low reflectivity

![image](https://github.com/user-attachments/assets/6362f437-40ea-4f12-8538-e3e70fc69beb)
![image](https://github.com/user-attachments/assets/72844ec6-470a-4b42-b251-8e0e0fad1de9)

<img width="600" alt="image" src="https://github.com/user-attachments/assets/6362f437-40ea-4f12-8538-e3e70fc69beb">
