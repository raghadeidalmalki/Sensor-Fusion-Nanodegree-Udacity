# LiDAR and Radar fusion with Kalman Filters, Extended Kalman Filter (EKF) and Unscented Kalman Filter 
## Kalman filters
Kalman filters provide estimations over a continuous state, which allows us to estimate future locations and velocities based on positional data.
In Kalman filters, the probability distribution is given by a **Gaussian**. A Gaussian is a unimodal continuous function over a space of inputs - locations, in this case. Like all probability distributions, the area underneath a Gaussian equals one. 

<img width="300" alt="image" src="https://github.com/user-attachments/assets/690be038-c800-4a29-ab08-d06ca24566ac">

A Gaussian is characterized by two parameters: 
-	Mean, μ (mu)

-	 Variance, $`σ^2`$.

Our task is to maintain a μ and $`σ^2`$ that parameterize the Gaussian that serves as our best estimate of the location of the object that we are trying to localize.

The expression inside the exponential shows that we are taking the squared difference of our query point, x, and our mean, μ, and dividing this squared difference by the variance, $`σ^2`$  ; the difference between x and μ is normalized by $`σ^2`$ . If x=μ, the numerator in this expression becomes zero, so we have exp(0) = 1. Indicating that the probability should be maximal when x equals the mean of the distribution.

Larger values of $`σ^2`$ indicate large differences between x and μ less than smaller values, in other words, high uncertainty. As a result, Gaussians with large variances produce larger values of f(x) when x is far from the mean than do Gaussians with smaller variances.

Kalmen filters iterate on two main cycles:
-	The first cycle is the **Measurement Update**:

Uses Bayes' rule, which produces a new posterior distribution by taking the product of the prior distribution and the information we gain from our measurement.

-	The second cycle is the **Motion Update**:

AKA the **prediction** - uses the theory of total probability, which produces a new posterior by adding the motion to the prior.

### Designing Kalman filter
When we design a Kalman filter we need 2 things:
1. State Transition Function `x′=Fx+Bu+ν` : models how the state has changed from time K minus one to time K. We will cross out `Bu`  leaving `x′=Fx+ν`
2. Measurement Function `Z = Hx' + w`: models how the measurement is calculated and how it's related to the predicted state x.

***`v (nu)` noise and `omega` noise represent the Stochastic part or in other words, are random noises that affect the prediction and measurement of the steps.***

`B` is a matrix called the **control input matrix** and `u` is the **control vector**.

As an example, let's say we were tracking a car and we knew for certain how much the car's motor was going to accelerate or decelerate over time; in other words, we had an equation to model the exact amount of acceleration at any given moment. Bu would represent the updated position of the car due to the internal force of the motor. We would use ν to represent any random noise that we could not precisely predict like if the car slipped on the road, or a strong wind moved the car.

For this lesson, we will assume that there is no way to measure or know the exact acceleration of a tracked object. For example, if we were in an autonomous vehicle tracking a bicycle, pedestrian or another car, we would not be able to model the internal forces of the other object; hence, we do not know for certain what the other object's acceleration is. Instead, we will set `Bu=0` and represent acceleration as a random noise with mean `ν`.



Other equations for the update and prediction steps can be found [here](https://github.com/raghadeidalmalki/Sensor-Fusion-Nanodegree-Udacity/blob/main/Kalman%20Filters%20/Kalman%20Filter%20Equations/sensor-fusion-ekf-reference.pdf)

## LiDAR and Radar fusion with Extended Kalman Filter (EKF)

The EKF is extended in a sense that it will be capable of handling more complex motion models and measurement models 

<img width="500" alt="image" src="https://github.com/user-attachments/assets/b87763b0-6ff6-43d6-8b1d-dff8235475ae">


The Kalman Filter algorithm will go through the following steps:

•	**First measurement** - the filter will receive initial measurements of the bicycle's position relative to the car. These measurements will come from a radar or lidar sensor.

•	**Initialize state and covariance matrices** - the filter will initialize the bicycle's position based on the first measurement.

•	Then the car will receive another sensor measurement after a time period Δt; each time we receive a new measurement from a given sensor the estimation function is triggered.

•	**Predict** - the algorithm will predict where the bicycle will be after time Δt, we predict the bicycle's state and its covariance, we do so by considering the elapsed time between the current and the previous observations. One basic way to predict the bicycle's location after Δt is to assume the bicycle's velocity is constant; thus the bicycle will have moved `velocity * Δt`. 

•	**Update** - the filter compares the "predicted" location with what the sensor measurement says. The predicted location and the measured location are combined to give an updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value. 

The measurement update step depends on the sensor type; if the current measurement is generated by a lidar sensor, we just apply a standard Kalman filter to update the  state. However, radar measurement involves a nonlinear measurement function, so when we receive radar measurement, we use different EKF equations to handle the measurement update.

•	Then the car will receive another sensor measurement after a time period Δt. The algorithm then does another **predict** and **update** step.


<img width="500" alt="image" src="https://github.com/user-attachments/assets/9675ad47-1a74-4179-abc1-a77139a4fd9d">


**Definition of Variables:** 

•	`x` is the **mean state vector**. For an extended Kalman filter, the mean state vector contains information about the object's position and velocity that you are tracking. It is called the "mean" state vector because position and velocity are represented by a gaussian distribution with mean `x`.

•	`P` is the **state covariance matrix**, which contains information about the uncertainty of the object's position and velocity. You can think of it as containing standard deviations.

•	`k` represents **time steps**. So `xk` refers to the object's position and velocity vector at time `k`.

•	The notation `k+1∣k` refers to the prediction step. At time `k+1`, you receive a sensor measurement. Before considering the sensor measurement to update your belief about the object's position and velocity, you predict where you think the object will be at time `k+1`. You can predict the position of the object at `k+1` based on its position and velocity at time `k`. Hence `xk+1∣k` means that you have predicted where the object will be at `k+1` but have not yet taken the sensor measurement into account.

•	`xk+1` means that you have now predicted where the object will be at time `k+1` and then used the sensor measurement to update the object's position and velocity.

### Radar and LiDAR Measurements
The state transition function is the same for both radar and lidar: 

<img width="200" alt="image" src="https://github.com/user-attachments/assets/3c382a9e-ce9d-44b6-b78b-cf79187663c8">


However, radar sees the world differently, radar can directly measure the following 

![image](https://github.com/user-attachments/assets/f1922400-d501-4fed-b5e1-a32cc6eb2597)

Therefor,

![image](https://github.com/user-attachments/assets/abd0b979-2514-42e9-857e-56df83ed5eb8)

**Definition of Radar Variables:**

•	The range, `(ρ)`, is the distance to the pedestrian. The range is basically the magnitude of the position vector `ρ` which can be defined as `ρ=sqrt(px^2+py^2)`.

•	`φ=atan(py/px)`. Note that `φ` is referenced counter-clockwise from the x-axis.

•	The range rate, `ρ˙`, is the projection of the velocity, v, onto the line, L.

The measurement function h of x’, that maps the predicted state x’ into the measurement space: 

![image](https://github.com/user-attachments/assets/bc96ee50-1cac-41cc-8f8f-c6298b9c0016)

![image](https://github.com/user-attachments/assets/0e932834-b3d0-400f-9ec5-8377f994b9e6)


This non-linear function specifies how the predictive position and speed can be related to the object range, bearing, and range rate.


**Deriving the Radar Measurement Function:**

The measurement function is composed of three components that show how the predicted state, x′=$`(px′,py′,vx′,vy′)^T`$, is mapped into the measurement space, z=$`(ρ,φ,ρ˙)^T`$:
The range, ρ, is the distance to the pedestrian which can be defined as:

ρ = $`\sqrt{px^2+py^2}`$

φ is the angle between ρ and the xx direction and can be defined as:

φ=$`\arctan(py/px)`$

There are two ways to do the range rate ρ(t)˙ derivation:
Generally we can explicitly describe the range, ρ, as a function of time:

ρ(t)= $`\sqrt{px(t)^2+py(t)^2}`$

The range rate, ρ(t)˙, is defined as time rate of change of the range, ρ, and it can be described as the time derivative of ρ:

 ![image](https://github.com/user-attachments/assets/3598b239-87a5-4d8b-93f6-bd07e94e07e3)


For simplicity we just use the following notation:


ρ˙= (Px Vx+Py Vy)/$`\sqrt{Px^2+Py^2}`$

The range rate, ρ˙, can be seen as a scalar projection of the velocity vector, **v**, onto **ρ**. Both ** ρ** and **v** are 2D vectors defined as:

![image](https://github.com/user-attachments/assets/ca804d18-32f4-4a49-9a1f-40473b9e2aba)

The scalar projection of the velocity vector **v** onto **ρ** is defined as:

![image](https://github.com/user-attachments/assets/154944de-4ca3-4055-a452-1545f31c5835)

where ∣**ρ**∣ is the length of **ρ**. In our case it is actually the range, so ρ=∣**ρ**|.

### Multivariate Taylor Series


As shown above the h function is composed of three equations that show how the predicted state, ′=$`(px′,py′,vx′,vy′)^T`$, is mapped into the measurement space, z=$`(ρ,φ,ρ˙)^T`$:

These are multi-dimensional equations, so we will need to use a multi-dimensional Taylor series expansion to make a linear approximation of the h function. Here is a general formula for the multi-dimensional Taylor series expansion:

![image](https://github.com/user-attachments/assets/8599d6a1-5d95-49ca-9448-a55d4128350a)

where Df(a) is called the Jacobian matrix and $`D^2`$f(a) is called the [Hessian matrix](https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/quadratic-approximations/a/the-hessian). They represent first order and second order derivatives of multi-dimensional equations. A full Taylor series expansion would include higher order terms as well for the third order derivatives, fourth order derivatives, and so on.

To derive a linear approximation for the h function, we will only keep the expansion up to the Jacobian matrix Df(a). We will ignore the Hessian matrix $`D^2`$f(a) and other higher order terms. Assuming (x−a)(x−a) is small, $`(x−a)^2`$ or the multi-dimensional equivalent $`(x−a)^T`$(x−a) will be even smaller; the extended Kalman filter we'll be using assumes that higher order terms beyond the Jacobian are negligible.

### EKF Equations and Kalman filters Equations 

The main differences are:

•	The `F` matrix will be replaced by `Fj` when calculating `P′`.

•	The `H` matrix in the Kalman filter will be replaced by the Jacobian matrix `Hj` when calculating `S`, `K`, and `P`.

•	To calculate `x′`, the prediction update function, `f`, is used instead of the `F` matrix. The predicted measurement vector x′ is a vector containing values in the form `[Px,Py,Vx,Vy]`. The radar sensor will output values in polar coordinates:

<img width="100" alt="image" src="https://github.com/user-attachments/assets/be17bae6-b757-4cc6-9d88-df94baf63ecb">


•	To calculate `y`, the `h` function is used instead of the `H` matrix. We use the equations that map the predicted location `x′` from Cartesian coordinates to polar coordinates:

<img width="500" alt="image" src="https://github.com/user-attachments/assets/adca5378-f499-4f8c-a81b-1c6fd06cf781">
