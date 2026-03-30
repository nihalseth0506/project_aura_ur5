# 🚀 Sprint 7: ROS2-Based Trajectory Execution & Real-Time Tracking

## 📌 Overview

This sprint bridges the gap between **simulation and real-world robotics execution**.

The optimal trajectory generated in MATLAB (Sprint 6) is now:

* Exported as Cartesian trajectory data  
* Executed in a ROS2-based control framework  
* Tracked in real-time using feedback control  
* Evaluated against the planned trajectory  

This transforms the system from:

**Offline optimization → Real-time execution and validation**

---

## 🎯 Objective

Develop a ROS2-based execution pipeline that:

* Follows MATLAB-generated optimal trajectories  
* Converts Cartesian motion into joint-space commands  
* Implements real-time feedback control  
* Ensures synchronized trajectory tracking  
* Quantifies tracking performance  

---

## 🧠 Methodology

### 1. Trajectory Export (MATLAB → ROS2)

The optimal trajectory is exported as:

* Position: `x(t) = [x, y, z]`  
* Velocity: `x_dot(t)`  

Velocity is computed using finite differences:

```bash
x_dot = diff(x) / dt
```

The final dataset:

```bash
[x, y, z, x_dot, y_dot, z_dot]
```

is saved as:

```bash
trajectory.csv
```

---

### 2. ROS2 Architecture

The system is implemented using two main nodes:

#### 🔹 Trajectory Planner Node

* Reads trajectory from CSV  
* Publishes desired position and velocity  
* Synchronizes with controller using a ready signal  

#### 🔹 Controller Node

* Receives desired trajectory  
* Computes control using Jacobian-based inverse kinematics  
* Applies feedback correction  
* Publishes:
  * Joint states  
  * End-effector pose  
  * Path for visualization  

---

### 3. Control Strategy

A velocity-based control law is used:

```bash
v = x_dot_desired + Kp * (x_desired - x_current)
```

Joint velocities are computed using a **damped pseudo-inverse Jacobian**:

```bash
J^+ = J^T (J J^T + λI)^(-1)
```

This ensures:

* Stability near singularities  
* Smooth motion  

---

### 4. Synchronization Mechanism

To ensure proper execution:

* Controller publishes `/controller_ready`  
* Planner sends next trajectory point only when ready  

This prevents:

* Skipping trajectory points  
* Timing mismatch  
* Unstable motion  

---

### 5. Initial Alignment

Before tracking begins:

* Robot first moves to the starting point  
* Trajectory execution starts only after alignment  

This ensures:

* Accurate path following  
* Proper comparison with MATLAB trajectory  

---

### 6. Real-Time Logging

The executed trajectory is recorded as:

```bash
ros_executed.csv
```

This contains:

* End-effector Cartesian positions  
* Computed using forward kinematics  

---

## 📊 Results

### Trajectory Tracking Comparison

* Planned trajectory (MATLAB) vs Executed trajectory (ROS2)  
* Strong alignment observed across full trajectory  

### Tracking Error

* Mean Error: ~0.0141 m  
* Max Error: ~0.0192 m  

The system demonstrates:

* Stable tracking  
* Smooth motion  
* Low deviation from planned path  

---

## 📸 Visualization

### MATLAB vs ROS2 Comparison

### Tracking Error

### RViz Execution

### ROS2 Architecture

---

## 🔍 Key Insights

* MATLAB-generated trajectories can be successfully executed in ROS2  
* Feedback control is essential for accurate tracking  
* Synchronization prevents instability in real-time systems  
* Initial alignment is critical for correct trajectory execution  
* Small tracking errors are expected due to discretization and control limits  

---

## 🏁 Conclusion

This sprint completes the transition from:

**Trajectory Optimization → Real-Time Robotic Execution**

The system now:

* Executes optimal trajectories in ROS2  
* Converts Cartesian commands to joint motion  
* Tracks motion using feedback control  
* Validates performance through error analysis  

This establishes a complete pipeline for:

**Simulation → Control → Execution → Validation**

---

# Author

**Nihal Sanjay Seth**  
Robotics and Control Systems Project  
2026