# Sprint 5 — UR5 Trajectory Tracking, Manipulability Monitoring, and Energy Analysis

This sprint implements a **full control and analysis framework for the UR5 robotic manipulator** using **MATLAB and Simulink**.

The system performs **Cartesian trajectory tracking while simultaneously analyzing energy consumption and kinematic manipulability**, providing insight into how different trajectories influence robot performance.

The simulator follows a **realistic robot control pipeline**, combining trajectory generation, inverse kinematics, dynamic control, and physics-based simulation.

---

# Objectives

The primary objectives of Sprint 5 were:

- Implement multiple Cartesian trajectories for the UR5 end-effector
- Track trajectories using a Cartesian control architecture
- Convert Cartesian commands to joint space using Jacobian inverse kinematics
- Apply dynamic control using a computed torque controller
- Monitor singularities using manipulability analysis
- Measure and compare energy consumption across trajectories
- Build a modular Simulink architecture resembling a real robot control system

---

# System Architecture

The simulator is organized into a modular control pipeline consisting of five main subsystems.

## Trajectory Generator

Generates desired end-effector motion in Cartesian space.

### Supported trajectories

- Circle
- Square
- Figure-8
- Spiral

### Outputs

```
xd -> desired end-effector position
xd_dot -> desired Cartesian velocity
```

The trajectory type can be switched interactively using a **Simulink knob input**.

---

## Robot Controller

The robot controller implements a Cartesian tracking pipeline.

```
Cartesian Controller
↓
Jacobian Inverse Kinematics
↓
Computed Torque Control
```

### Cartesian Controller

Computes Cartesian velocity commands based on position error.

```
e = xd - x
```

The error is used to generate desired end-effector velocity.

---

### Jacobian Inverse Kinematics

Converts Cartesian velocity commands into joint velocities using **damped least-squares inverse kinematics**.

```
q_dot = J^T ( J J^T + lambda^2 I )^-1 v
```

Where:

- `J` is the robot Jacobian
- `lambda` is the damping factor

This improves numerical stability near singularities.

---

### Computed Torque Control

Joint torques are computed using the robot dynamic model.

```
tau = M(q) q_ddot + C(q,q_dot) q_dot + G(q)
```

Where:

- `M(q)` is the mass matrix
- `C(q,q_dot)` is the Coriolis matrix
- `G(q)` is the gravity vector

A **torque saturation block** ensures physically reasonable torques.

---

## UR5 Plant

The plant subsystem simulates the physical dynamics of the robot.

### Inputs

```
tau -> joint torques
```

### Outputs

```
q -> joint positions
q_dot -> joint velocities
```

This represents the **true robot motion produced by the control system**.

---

## Robot Sensors

Implements forward kinematics to compute the end-effector position.

### Input

```
q
```

### Output

```
x -> end-effector position
```

This closes the feedback loop for Cartesian control.

---

## Energy Analyzer

Energy consumption is evaluated using joint torque and velocity.

Instantaneous power is computed as:

```
P = | tau * q_dot |
```

Energy is obtained by integrating power over time:

```
E = integral P dt
```

### Outputs

```
energy_log
```

This allows quantitative comparison of trajectories based on energy efficiency.

---

# Manipulability Monitoring

Robot manipulability is computed from the singular values of the Jacobian.

```
w = product(sigma_i)
```

Where `sigma_i` are the singular values of the Jacobian.

Low manipulability values indicate proximity to **kinematic singularities**.

A warning lamp in the Simulink model activates when the manipulability drops below a predefined threshold.

---

# Implemented Trajectories

Four Cartesian trajectories were implemented for analysis.

## Circle

Smooth circular motion with constant radius.

## Square

Piecewise linear motion with sharp direction changes.

## Figure-8

Continuous trajectory with alternating curvature.

## Spiral

Expanding spiral path covering a larger workspace region.

Each trajectory generates different:

- joint velocities
- torque requirements
- energy consumption
- manipulability behavior

---

# Results

Three types of results are generated for analysis.

## Trajectory Tracking

Desired and actual end-effector trajectories are plotted using an XY graph.

This provides visual confirmation of tracking accuracy and controller performance.

### Simulation Videos

Due to GitHub file size limits, the full simulation recordings are hosted externally.

📹 Sprint 5 Simulation Demonstrations  
https://drive.google.com/drive/folders/1pYAJ0i50DlaDnV5hHvAa-gQohSg_l6TZ?usp=drive_link

---

## Energy Consumption Comparison

The total energy required for each trajectory is computed and compared.

Example observations:

- Smooth trajectories generally require less energy
- Trajectories with sharp direction changes increase torque demands
- Large workspace motions increase total energy consumption

---

## Manipulability Analysis

Manipulability is plotted during trajectory execution.

This reveals how different trajectories move the robot through regions of varying dexterity.

Low values indicate configurations approaching singularities.

---

# Simulink Model Structure

The final Simulink architecture consists of the following subsystems:

```
Trajectory_Generator
Robot_Controller
UR5_Plant
Robot_Sensors
Energy_Analyzer
```

Additional components include:

- torque saturation
- trajectory selection knob
- singularity warning indicator
- energy logging
- trajectory visualization

The structure closely mirrors **real industrial robot control pipelines**.

---

# Key Features

- Modular robot control architecture
- Multiple Cartesian trajectory generators
- Jacobian inverse kinematics with damping
- Dynamic control using computed torque control
- Singularity detection via manipulability monitoring
- Energy consumption measurement and comparison
- Interactive trajectory selection
- Realistic robot dynamics simulation

---

# Significance

This sprint establishes a **physics-based robot simulation framework** capable of analyzing the interaction between:

- trajectory design
- robot dynamics
- energy efficiency
- kinematic dexterity

The architecture forms a strong foundation for advanced robotics research topics including:

- energy-optimal trajectory planning
- manipulability-aware motion planning
- obstacle avoidance
- ROS2 integration
- digital twin development

---

# Author

**Nihal Sanjay Seth**  
Robotics and Control Systems Project  
2026