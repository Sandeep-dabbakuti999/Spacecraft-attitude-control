###Quaternions
Quaternion represents rotation by a rotational angle around a rotational axis.The quaternion is represented as the sum of scalar and vector

$\vec{q}=q_o+q_1*i+q_2*j+q_3*k$

###Equation of Motion of a Space Craft
The cost of using only three components of the quaternion in the model is that, similar to the Euler angle representation, the reduced model has a singular point at α = ±π, where α 
is the rotation angle around the rotation axis.However, this singular point is the farthest point to the point where the linearization is carried out. The stability of the designed 
closed-loop spacecraft system is guaranteed because the linearized control system is fully controllable.

##Dynamics equation of a spacecraft
The Dynamics of a spacecraft with external torque T applied on spacecraft due to some external disturbances like gravitational,aerodynamic,solar radiation and other environmental tor
-ques is denoted by t_d and control torque u.

#The following resultant equation
${dy \over dx}=-w_I x(J*w_I)+t_d+u$

#The Kinematic Equation of Spacecraft
The kinematic equation of spacecraft gives out the resulting quaternion. 
${dq_-\over dt=q(t)x(0.5*w(t))}$

<img src="images\Dynamics.png" width="300" />

Using a linearized reduced quaternion model, we can derive an analytical formula for LQR optimal control that is explicitly related to the cost matrices Q and R. The LQR feedback con
-troller globally stabilizes the original nonlinear spacecraft.

The linear system given as
$x_.=A*x+B*u$
$y=C*x$
The feedback matrix for the LQR to minimize the objective function O
$O=1/2*\int_0^\infty (x^TQx)+(u^T*R*u)\,dt$

<img src="images\control.png" width="300" />
