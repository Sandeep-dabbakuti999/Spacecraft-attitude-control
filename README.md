# Quaternions

A **quaternion** represents a rotation. In scalar–vector form:


q = q0 +q1*i +q2*j +q_3*k


## Equation of Motion of a Spacecraft

The cost of using only three components of the quaternion in the model is that, similar to the Euler angle representation, the reduced model has a singular point at α = ±π, where α is the rotation angle around the rotation axis.However,
this singular point is the farthest point to the point where the linearization is carried out. The stability of the designed closed-loop spacecraft system is guaranteed because the linearized control system is fully controllable.

---

### Dynamics of a Spacecraft

The Dynamics of a spacecraft with external torque T applied on spacecraft due to some external disturbances like gravitational,aerodynamic,solar radiation and other environmental tor -ques is denoted by t_d and control torque u.
The following resultant equation:

${dy \over dx}=-w_I x(J*w_I)+t_d+u$

---

### Kinematics (Quaternion Rate)

The kinematic equation of spacecraft gives out the resulting quaternion. 

${dq_-\over dt=q(t)x(0.5*w(t))}$

<img src="Images/Kinematics&Din.png" width="300" />

using a linearized reduced quaternion model, we can derive an analytical formula for LQR optimal control that is explicitly related to the cost matrices Q and R. The LQR feedback controller globally stabilizes the original nonlinear spacecraft.

---

## Linearized LQR Design

The linear system is given as 

x=A*x+B*u,
y=C*x 

The feedback matrix for the LQR to minimize the objective function O

$$
O = \tfrac{1}{2} \int_0^{\infty} \left( x^T Q x + u^T R u \right)\, dt
$$

<img src="Images\TOT.png" width="300" />
