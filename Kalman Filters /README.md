

Deriving the Radar Measurement Function
The measurement function is composed of three components that show how the predicted state, $`\x′=(px′,py′,vx′,vy′)^T`$, is mapped into the measurement space, $`\z=(ρ,φ,ρ˙)^T`$:
The range, ρ, is the distance to the pedestrian which can be defined as:

ρ = $`\sqrt{px^2+py^2}`$

φ is the angle between ρ and the xx direction and can be defined as:

$`\φ=arctan(py/px)`$

There are two ways to do the range rate ρ(t)˙ derivation:
Generally we can explicitly describe the range, ρ, as a function of time:

$`\ρ(t)=sqrt{px(t)^2+py(t)^2}`$

The range rate, ρ(t)˙, is defined as time rate of change of the range, ρ, and it can be described as the time derivative of ρ:
 

For simplicity we just use the following notation:


$`\ρ˙= PxVx+PyVy/sqrt{Px^2+Py^2}`$
