# Project AURA  
## Autonomous Unified Resource-Optimized Actuation  

**Energy-Optimized Motion Planning and Control Framework for a 6-DOF Industrial Manipulator**

---

## Overview

Project AURA is a structured robotics engineering initiative focused on developing a singularity-aware, energy-efficient motion planning and control framework for a 6-DOF UR5 industrial manipulator.

The project is executed in clearly defined technical sprints, progressing from foundational kinematic modeling toward robust differential control strategies.

---

# Sprint 1 – Kinematic Modeling & Workspace Validation ✅

## Objectives

- Implement full Denavit–Hartenberg (DH) parameter-based kinematic model  
- Derive complete forward kinematics (T₀⁶)  
- Validate rotation matrix orthonormality  
- Generate Monte Carlo-based workspace estimation  
- Perform singularity detection using Jacobian rank analysis  

## Outcomes

- Modular DH transformation framework implemented  
- Verified forward kinematics consistency  
- Physically accurate workspace envelope (~0.93 m reach)  
- Singularity identification through rank deficiency  
- Deterministic simulation structure for reproducibility  

Sprint 1 established the mathematical foundation of the manipulator model.

---

# Sprint 2 – Differential Kinematics & Singularity-Robust Control ✅

Sprint 2 extended the project from static kinematics to velocity-level control.

## Objectives

- Derive and implement the 6×6 geometric Jacobian  
- Solve the inverse velocity problem  
- Implement Damped Least Squares (DLS)  
- Implement manipulability-based adaptive damping  
- Compare safe vs singular configurations  
- Evaluate circular Cartesian tracking performance  

---

## Key Implementations

### Jacobian Matrix

- Analytical 6×6 Jacobian derived from the forward kinematic chain  
- Verified rank behavior across configurations  
- Used for velocity mapping between joint space and Cartesian space  

### Inverse Velocity Control

- Implemented Damped Least Squares to prevent instability  
- Compared fixed damping vs adaptive damping strategies  

### Manipulability Analysis

- SVD-based manipulability index  
- Real-time tracking of dexterity across motion  
- Automatic damping increase near singular configurations  

---

## Experimental Validation

Circular Cartesian tracking experiments were conducted under:

- Safe configuration  
- Near-singular configuration  
- Fixed damping  
- Adaptive damping  

### Observations

- Fixed damping → better tracking accuracy but higher joint velocity near singularities  
- Adaptive damping → improved stability with reduced velocity spikes  
- Manipulability index correctly predicts proximity to singularity  
- Adaptive damping increases automatically in low-dexterity regions  

Sprint 2 demonstrates singularity-aware motion control behavior.

---

# Sprint 3 – Inverse Dynamics & Energy Optimization ✅

Sprint 3 extends the framework from velocity-level control to full rigid-body dynamic modeling and energy analysis.

The objective was to evaluate actuation effort, power flow, and trajectory efficiency under varying motion conditions.

## Objectives

- Implement full inverse dynamics model  
- Compute joint torques using rigid-body dynamics  
- Incorporate mass matrix M(q), Coriolis matrix C(q, q̇), and gravity vector G(q)  
- Analyze mechanical power and total energy consumption  
- Perform trajectory radius scaling study  
- Compare Fixed vs Adaptive damping from an energy perspective  

---

## Key Implementations

### Inverse Dynamics Model

The manipulator torque model was implemented as:

τ = M(q)q̈ + C(q,q̇)q̇ + G(q)

Where:

- M(q) → configuration-dependent mass matrix  
- C(q,q̇) → Coriolis and centrifugal effects  
- G(q) → gravity torque vector  

Joint accelerations were computed numerically from joint velocity history.

---

### Mechanical Power & Energy Computation

For each motion:

- Instantaneous mechanical power computed as:

  P = τᵀ q̇

- Total mechanical energy computed via numerical integration  

This enabled direct evaluation of actuation effort during trajectory execution.

---

### Radius Scaling Study

Circular trajectories were evaluated for:

r ∈ {0.03, 0.1, 0.2, 0.3} m

For each radius and damping strategy:

- Total mechanical energy  
- RMS joint torque norm  
- RMS mechanical power  
- Energy per meter traveled  
- RMS Cartesian tracking error  

---

## Experimental Observations

- Torque demand increases with trajectory radius  
- Power consumption grows nonlinearly with motion amplitude  
- Energy per meter reveals efficiency degradation at larger radii  
- Adaptive damping improves tracking robustness  
- Fixed damping can be more energy-efficient in well-conditioned regions  
- Shoulder joint (τ₂) dominates gravitational loading  

Detailed dynamic visualization includes:

- Joint torque norm over time  
- Mechanical power profile  
- Individual joint torque contributions  

---

Sprint 3 establishes a complete energy-aware dynamic evaluation framework.

---

## Author

**Nihal Sanjay Seth**

---

## Project Status

Version: v3.0  
Sprint 1 – Complete  
Sprint 2 – Complete  
Sprint 3 - Complete