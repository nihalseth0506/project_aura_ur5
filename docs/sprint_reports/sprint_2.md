# 🚀 Sprint 2 — Jacobian Control, Singularities & Adaptive Damping (UR5)

## Overview

Sprint 2 extends the UR5 kinematic workspace analysis from Sprint 1 into **velocity-level Cartesian control**.

In this sprint, we implemented:

- Jacobian-based differential inverse kinematics
- Manipulability analysis using SVD
- Damped Least Squares (DLS)
- Adaptive damping based on manipulability
- Circular Cartesian trajectory tracking
- Safe vs Singular configuration comparison

This sprint focuses on understanding **how robots behave near singularities** and how damping stabilizes motion.

---

# 1️⃣ Mathematical Foundation

## Differential Kinematics

\[
x_dot = J(q) * q_dot
\]
Where:

- x_dot → End-effector twist (linear + angular velocity)
- J(q) → 6×6 Jacobian
- q_dot → Joint velocities

---

## Inverse Velocity Problem

\[
q_dot = J^{-1} * x_dot
\]
⚠️ This fails at singularities.

---

## Damped Least Squares (DLS)

\[
q_dot = J^T * (J J^T + λ^2 I)^{-1} * x_dot
\]

- λ → damping factor
- Prevents velocity explosion near singularity
- Trades accuracy for stability

---

## Manipulability Index

Using SVD:

\[
w = σ1 * σ2 * σ3 * σ4 * σ5 * σ6
\]

Where:
- σi are singular values of J

Interpretation:

- High \( w \) → good dexterity
- \( w → 0 \) → singular configuration

---

# 2️⃣ Adaptive Damping Strategy

Instead of constant λ:

\[
λ = λ_min + k / (w + ε)
\]

Behavior:

- Far from singularity → small damping
- Near singularity → large damping
- Saturated using λ_max

This enables **automatic stabilization**.

---

# 3️⃣ Experiments Performed

We ran 4 controlled experiments:

| Case | Configuration | Damping |
|------|--------------|---------|
| 1 | Safe | Fixed |
| 2 | Singular | Fixed |
| 3 | Safe | Adaptive |
| 4 | Singular | Adaptive |

Trajectory:  
3 cm circular Cartesian path.

---

# 4️⃣ Results Summary

## Safe Configuration

- Manipulability high
- Tracking accurate
- Low joint velocity spikes
- Adaptive ≈ Fixed

---

## Singular Configuration

- Manipulability collapses
- Fixed damping → velocity spikes
- Adaptive damping → stabilized motion
- Tracking accuracy reduced but safe

Key observation:

> Adaptive damping sacrifices tracking accuracy to preserve numerical stability and physical feasibility.

---

# 5️⃣ Visual Outputs

Generated Figures:

- `manipulability_map_sprint2.png`
- `safe_config_comparison_sprint2.png`
- `singular_config_comparison_sprint2.png`
- `manipulability_index_sprint2.png`
- `circle_tracking_comparison_sprint2.png`

These demonstrate:

- Dexterity variation in workspace
- Velocity spikes near singularity
- Effect of damping on control behavior

---

# 6️⃣ Engineering Insights

✔ Singularities are unavoidable in serial manipulators  
✔ Pure Jacobian inversion is unsafe  
✔ DLS provides robustness  
✔ Adaptive damping improves stability  
✔ Manipulability is a practical singularity metric  
✔ Stability–accuracy tradeoff is fundamental in robotics control  

---

# 7️⃣ What This Means Practically

In real industrial robots:

- Controllers avoid singular regions
- Path planners consider manipulability
- Damping is dynamically adjusted
- Joint velocity limits are enforced

This sprint models the **core internal logic of industrial robot controllers** at the velocity level.

---

# 9️⃣ Version

Sprint 2 complete → Tag as:

v2.0 — Jacobian Control & Adaptive Damping

---

# 🔜 Next Direction (Sprint 3)

Possible extensions:

- Full pose (position + orientation) control
- Manipulability ellipsoid visualization
- Null-space optimization
- Joint limit avoidance
- 3D UR5 animation model
- ROS 2 integration

---

## Conclusion

Sprint 2 transforms this project from static kinematics to **dynamic Jacobian-based control with singularity handling**.

This is a major step toward:
- Real robot controller architecture
- Advanced robotics research topics
- Industrial-grade motion control understanding
