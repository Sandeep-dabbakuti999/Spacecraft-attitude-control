# Quaternions

A **quaternion** represents a rotation. In scalar–vector form:
\[
\mathbf{q} = q_0 + \mathbf{q}_v = q_0 + q_1 i + q_2 j + q_3 k,
\qquad
\mathbf{q}_v = \begin{bmatrix} q_1 \\ q_2 \\ q_3 \end{bmatrix}.
\]

---

## Equation of Motion of a Spacecraft

Using a reduced quaternion model introduces a singularity when the rotation angle \(\alpha = \pm\pi\). This singularity is far from the small-angle linearization point, and the linearized control design can still guarantee stability when the linearized model is fully controllable.

---

### Dynamics of a Spacecraft

The rotational dynamics (Euler's equation) are:
\[
\dot{\boldsymbol{\omega}} = J^{-1}\Big(-\boldsymbol{\omega}\times(J\boldsymbol{\omega}) + \boldsymbol{\tau}_d + \mathbf{u}\Big),
\]
where
- \(\boldsymbol{\omega}\) is the angular velocity (body frame),
- \(J\) is the inertia matrix,
- \(\boldsymbol{\tau}_d\) is the disturbance torque,
- \(\mathbf{u}\) is the control torque.

---

### Kinematics (Quaternion Rate)

The quaternion time-derivative can be written using the angular velocity as:

**Quaternion ordinary form**
\[
\dot{\mathbf{q}} = \tfrac{1}{2}\,\mathbf{\Omega}(\boldsymbol{\omega})\,\mathbf{q},
\]
where \(\mathbf{q}=[q_0,\;q_1,\;q_2,\;q_3]^\top\) and
\[
\mathbf{\Omega}(\boldsymbol{\omega})
=
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\[4pt]
\omega_x & 0 & \omega_z & -\omega_y\\[4pt]
\omega_y & -\omega_z & 0 & \omega_x\\[4pt]
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}.
\]

**Scalar–vector split (equivalent)**
\[
\dot{q}_0 = -\tfrac{1}{2}\,\mathbf{q}_v\cdot\boldsymbol{\omega},\qquad
\dot{\mathbf{q}}_v = \tfrac{1}{2}\!\left(q_0\boldsymbol{\omega} + \mathbf{q}_v\times\boldsymbol{\omega}\right).
\]

---

<img src="images/Dynamics.png" width="300" />

---

## Linearized LQR Design

Linearized state-space:
\[
\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u},\qquad
\mathbf{y} = C\mathbf{x}.
\]

LQR cost to minimize:
\[
J = \tfrac{1}{2}\int_0^{\infty}\!\left(\mathbf{x}^\top Q \mathbf{x} + \mathbf{u}^\top R \mathbf{u}\right)\,dt.
\]

The optimal feedback is \(\mathbf{u} = -K\mathbf{x}\) with \(K = R^{-1}B^\top P\), where \(P\) solves the algebraic Riccati equation.

<img src="images/control.png" width="300" />

