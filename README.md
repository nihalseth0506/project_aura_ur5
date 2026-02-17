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

## Author

**Nihal Sanjay Seth**

---

## Project Status

Version: v2.0  
Sprint 1 – Complete  
Sprint 2 – Complete  
