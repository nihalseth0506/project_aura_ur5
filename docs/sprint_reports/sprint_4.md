# 🚀 Sprint 4 – Payload-Aware Dynamic Pick & Place Analysis

---

## 📌 Objective

Extend the UR5 pick-and-place simulation to include:

- Dynamic inverse dynamics with payload  
- Real-time payload switching (grip/release)  
- Torque distribution analysis  
- Energy consumption comparison  
- Professional visualization and video export  

---

## 🏗 System Architecture

This sprint builds on previous sprints:

- Forward kinematics (DH-based)  
- Jacobian-based inverse kinematics  
- Damped least squares control  
- Joint velocity & position constraints  
- Full inverse dynamics model  

### 🆕 New Additions in Sprint 4

- Payload-aware inverse dynamics  
- Absolute mechanical energy computation  
- Torque comparison (load vs no-load)  
- Energy increase quantification  
- Table environment modeling  
- C-shaped gripper visualization  
- Automated graph + video export  

---

## ⚙️ Control Strategy

### Cartesian Closed-Loop Controller

x_dot = x_dot_ff + K * e

where:
- x_dot      → Cartesian velocity
- x_dot_ff   → feedforward velocity
- K          → feedback gain matrix
- e          → Cartesian position error

### Joint Velocity Computation (Damped Least Squares)

q_dot = J^T * (J * J^T + lambda^2 * I)^(-1) * v

where:
- J          → Jacobian matrix
- lambda     → damping factor
- I          → identity matrix
- v          → desired Cartesian velocity

Notes:
- Adaptive damping based on Jacobian singular values
- Ensures stable behavior near singular configurations

---

## ⚖️ Payload Switching Logic

During task execution:

- `"grip"` → payload mass = **5 kg**  
- `"release"` → payload mass = **0 kg**

Inverse dynamics are recalculated at **every timestep**.

---

## 🔋 Energy Calculation

Absolute mechanical energy:

E = sum( | tau^T * q_dot | * dt )

This reflects realistic motor energy consumption  
(no cancellation during braking).

---

## 📊 Results

### Total Energy Comparison

| Case           | Energy (J) |
|---------------|------------|
| No Payload     | 25.751 J   |
| With Payload   | 33.598 J   |

### 🔺 Energy Increase

\[
Increase = 30.47\%
\]

---

## 📈 Torque Observations

- Joint 2 & Joint 3 (shoulder/elbow) show highest increase  
- Wrist joints minimally affected  
- Load influence visible only during transport phase  
- Base joint nearly unchanged  

This matches expected industrial manipulator behavior.

---

## 🎥 Deliverables

Saved automatically to:
media/images/sprint4/

- `torque_norm_comparison.png`
- `joint_torque_comparison.png`
- `energy_comparison.png`
- `sprint4_animation.avi`

---

## 🧠 Engineering Insight

Sprint 4 demonstrates:

- Proper integration of dynamics + control  
- Realistic gravity compensation effects  
- Quantitative energy performance evaluation  
- Modular architecture suitable for scaling  

This moves the project from **visualization → engineering analysis**.

---

# 🏁 Sprint 4 Completed

**Status:** ✅ Complete  
**Version:** v4  
**Ready for push and documentation**

---

# 🔜 Next: Sprint 5 Ideas

Possible directions:

- Moving conveyor object tracking  
- Collision detection  
- Trajectory optimization  
- Time-optimal motion  
- Energy minimization study  

---