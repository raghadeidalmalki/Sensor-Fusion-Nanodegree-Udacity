# Kaman Filter, Extended Kalman Filter (EKF), Unscented Kalman Filter and LiDAR and Radar fusion with Kalman filters
## Kalman filters
Kalman filters provide estimations over a continuous state, which allows us to estimate future locations and velocities based on positional data.
In Kalman filters, the probability distribution is given by a **Gaussian**. A Gaussian is a unimodal continuous function over a space of inputs - locations, in this case. Like all probability distributions, the area underneath a Gaussian equals one. 

![image](https://github.com/user-attachments/assets/690be038-c800-4a29-ab08-d06ca24566ac)

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
1. State Transition Function: models how the state has changed from time K minus one to time K.
2. Measurement Function: models how the measurement is calculated and how it's related to the predicted state x.







----------------


Deriving the Radar Measurement Function
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
