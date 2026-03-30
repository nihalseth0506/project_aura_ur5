# Project AURA  
## Autonomous Unified Resource-Optimized Actuation  

**Energy-Optimized Motion Planning and Control Framework for a 6-DOF Industrial Manipulator**

---

## Overview

Project AURA is a structured robotics engineering initiative focused on developing a singularity-aware, energy-efficient motion planning and control framework for a 6-DOF UR5 industrial manipulator.

The project is executed in clearly defined technical sprints, progressing from foundational kinematic modeling toward real-time ROS2-based trajectory execution and validation.

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

# Sprint 4 – Payload-Aware Pick & Place Energy Analysis ✅

Sprint 4 transitions Project AURA from trajectory-based dynamic evaluation to **task-level manipulation analysis under variable payload conditions**.

The focus shifts from circular motion studies to a realistic industrial pick-and-place operation with dynamic payload switching.

---

## Objectives

- Implement task-level Cartesian pick-and-place execution  
- Integrate real-time payload switching (grip / release)  
- Extend inverse dynamics to include external payload mass  
- Compare torque distribution with and without load  
- Quantify mechanical energy increase due to payload  
- Build full simulation environment (tables, object, gripper)  
- Export reproducible graphs and video documentation  

---

## System Extension

### Payload-Aware Inverse Dynamics

During task execution:

- At **"grip" phase** → payload mass activated  
- At **"release" phase** → payload mass removed  

The dynamic model becomes:

τ = M(q)q̈ + C(q,q̇)q̇ + G(q) + τ_payload

This enables:

- Realistic gravity compensation  
- Proper torque redistribution across joints  
- Task-phase dependent dynamic behavior  

---

## Energy Computation Strategy

Absolute mechanical energy was computed as:

P = |τᵀ q̇|  
E = ∑ P dt  

Using absolute power prevents cancellation during deceleration phases and reflects realistic actuator energy consumption.

---

## Experimental Setup

### Task Phases

1. Approach pick  
2. Lower  
3. Grip  
4. Lift  
5. Transport  
6. Lower  
7. Release  
8. Return  

**Payload mass:** 5 kg  
**Controller:** Adaptive Damped Least Squares  
**Sampling time:** 0.02 s  

---

## Results

### Total Energy Consumption

| Case         | Energy (J) |
|-------------|------------|
| No Payload  | 25.751 J   |
| With Payload| 33.598 J   |

### Energy Increase

+30.47%

---

## Torque Distribution Observations

- Shoulder joints (τ₂, τ₃) show dominant increase  
- Base joint minimally affected  
- Wrist joints nearly unchanged  
- Load influence appears primarily during transport phase  
- Gravitational effects visible in torque plateaus  

This aligns with expected industrial manipulator behavior.

---

## Deliverables

Automatically exported to:
media/images/sprint4/

- Torque norm comparison  
- Individual joint torque comparison  
- Energy comparison bar graph  
- Full pick-and-place simulation video  

---

## Engineering Significance

Sprint 4 establishes:

- Task-level dynamic validation  
- Quantified energy impact of payload  
- Modular architecture for external force modeling  
- Transition from motion feasibility → task-aware energy evaluation  

Project AURA now integrates:

Kinematics → Differential Control → Full Dynamics → Task-Level Energy Analysis

---

# Sprint 5 – Multi-Trajectory Tracking & Energy–Dexterity Analysis ✅

Sprint 5 extends Project AURA from single-task motion execution to **multi-trajectory comparative analysis**, enabling evaluation of how different motion geometries affect **robot energy consumption and kinematic dexterity**.

The focus shifts from payload-dependent task execution toward **trajectory-dependent dynamic behavior**, while maintaining the full control and dynamics pipeline developed in previous sprints.

---

## Objectives

- Implement a modular **multi-trajectory generator**
- Execute Cartesian trajectory tracking for multiple path geometries
- Monitor **manipulability during motion**
- Compare **energy consumption across trajectories**
- Integrate **singularity detection with real-time warning**
- Build a clean **Simulink control architecture** representing a realistic robot control stack

---

## System Extension

### Multi-Trajectory Generator

Four Cartesian trajectories were implemented:

- Circle  
- Square  
- Figure-8  
- Spiral  

A Simulink knob allows **interactive switching between trajectory types** during simulation.

Each trajectory produces different:

- velocity profiles
- joint torques
- energy consumption
- manipulability behavior

---

### Cartesian Tracking Pipeline

The complete control pipeline implemented in previous sprints was reused:

Cartesian Controller  
→ Jacobian Inverse Kinematics (Damped Least Squares)  
→ Computed Torque Control  
→ UR5 Dynamic Plant  

This structure ensures that the robot motion arises from **true dynamic simulation rather than scripted animation**.

---

### Manipulability Monitoring

Robot dexterity is evaluated using the Jacobian singular values:

w = ∏ σᵢ

Low manipulability indicates proximity to **kinematic singularities**.

A **real-time warning indicator** activates when the robot approaches low-dexterity regions.

---

### Energy Evaluation

Mechanical power is computed as:

P = |τᵀ q̇|

Total energy:

E = ∑ P dt

This enables direct comparison of **trajectory energy efficiency**.

---

## Experimental Observations

Comparative analysis of trajectories revealed:

- Smooth trajectories require **lower torque variation**
- Sharp path transitions increase **energy demand**
- Large workspace coverage increases **actuation effort**
- Manipulability varies significantly across trajectories
- Spiral trajectories explore wider workspace regions and show **larger energy consumption**

Energy comparison and manipulability trends were exported as reproducible plots.

---

## Deliverables

Automatically exported to:

media/images/sprint5/

- Trajectory energy comparison plot  
- Manipulability comparison across trajectories  
- Energy vs time curves  
- Full multi-trajectory simulation video  

---

## Engineering Significance

Sprint 5 establishes a **trajectory-aware analysis framework** where robot performance can be evaluated based on motion geometry.

Project AURA now integrates:

Kinematics → Differential Control → Dynamics → Task-Level Analysis → Trajectory-Level Energy & Dexterity Evaluation

This framework forms the basis for future research directions such as:

- energy-optimal trajectory planning  
- manipulability-aware motion planning  
- obstacle avoidance strategies  
- digital twin integration  

---

# Sprint 6 – Trajectory Optimization with Energy–Manipulability Trade-off & Adaptive Sampling ✅

Sprint 6 advances Project AURA from trajectory analysis to **trajectory selection and optimization**.

Instead of executing predefined trajectories, the system now **generates, evaluates, filters, and optimizes multiple candidate paths** based on energy efficiency, kinematic dexterity, and physical feasibility.

The framework evolves from motion execution to **intelligent trajectory optimization under physical and kinematic constraints**, bridging the gap between simulation and real-world robotic decision-making.

---

## Objectives

- Generate multiple candidate trajectories between the same start and goal
- Evaluate trajectories using **energy and manipulability metrics**
- Implement **constraint-based rejection** (singularity & torque limits)
- Formulate a **time-integrated cost function**
- Select optimal trajectory under:
  - Energy-only criterion
  - Energy + Manipulability criterion
- Implement **adaptive sampling** to refine trajectory quality
- Visualize valid, rejected, and optimal trajectories

---

## System Extension

### Trajectory Sampling

Two modes are implemented:

#### Fixed Mode
- Line  
- Arc  
- Spline  

#### Random Mode
- Multiple trajectories generated using **quadratic Bézier curves**
- Each trajectory defined by a random control point:

x(t) = (1 − τ)²P₀ + 2(1 − τ)τP₁ + τ²P₂

Where:
- P₀ = start  
- P₁ = control point  
- P₂ = goal  
- τ = normalized time  

---

### Constraint-Based Filtering

Each trajectory is validated using:

- **Manipulability threshold**
  
  w < 0.0015 → REJECTED  

- **Torque limit**

  |τ| > 150 → REJECTED  

Rejected trajectories are excluded from optimization but remain visible in visualization.

---

### Cost Function Formulation

A **time-integrated cost function** is used:

J = ∫ ( |P(t)| + λ / w(t) ) dt

Where:

- P(t) = dE/dt → instantaneous power  
- w(t) = manipulability  
- λ = weighting factor  

This ensures:

- Energy-efficient motion  
- Avoidance of singular configurations  

---

### Adaptive Sampling (Key Innovation)

After initial evaluation:

1. Best trajectory (based on cost) is selected  
2. Its control point is extracted  
3. New trajectories are generated locally:

P₁(new) = P₁(best) + noise  

This results in:

- Local search around optimal region  
- Improved trajectory refinement  
- Reduced dependence on random sampling  

---

## Visualization Strategy

- **Valid trajectories** → solid lines  
- **Rejected trajectories** → dashed grey  
- **Adaptive trajectories** → dotted blue  
- **Optimal trajectory** → bold dashed black  

Two comparisons are shown:

- Energy-optimal path  
- Energy + Manipulability optimal path  

---

## Experimental Observations

- Energy-optimal paths may pass through **low manipulability regions**  
- Cost-based optimization produces **safer and more stable trajectories**  
- Adaptive sampling leads to **trajectory clustering around optimal region**  
- Constraint filtering prevents unsafe motion execution  
- Manipulability plays a critical role in real-world feasibility  

---

## Deliverables

Automatically exported to:

docs/images/sprint6/

- Fixed trajectory comparison  
- Random + adaptive trajectory optimization  
- Optimal path visualization with constraints  

---

## Engineering Significance

Sprint 6 represents a major transition:

Trajectory Execution → Trajectory Optimization

The system now:

- Generates multiple motion candidates  
- Filters infeasible trajectories  
- Optimizes based on performance metrics  
- Refines solutions using adaptive sampling  

---

Project AURA now integrates:

Kinematics → Differential Control → Dynamics → Task-Level Analysis → Trajectory Analysis → **Trajectory Optimization**

---

# Sprint 7 – ROS2-Based Trajectory Execution & Real-Time Tracking ✅

Sprint 7 completes Project AURA by transitioning from offline trajectory optimization to **real-time robotic execution and validation using ROS2**.

The optimal trajectory generated in Sprint 6 is exported and executed through a ROS2 control pipeline, enabling **closed-loop tracking, synchronization, and performance evaluation**.

---

## Objectives

- Execute MATLAB-generated optimal trajectories in ROS2  
- Convert Cartesian trajectories into joint-space motion  
- Implement real-time feedback control  
- Ensure synchronized trajectory execution  
- Validate tracking accuracy using quantitative metrics  

---

## System Architecture

The ROS2 system consists of two core nodes:

### Trajectory Planner Node
- Reads optimal trajectory from CSV  
- Publishes desired position and velocity  
- Waits for controller readiness before sending next point  

### Controller Node
- Receives desired trajectory  
- Computes joint velocities using Jacobian-based inverse kinematics  
- Applies feedback correction  
- Publishes:
  - Joint states  
  - End-effector pose  
  - Path for visualization  

A synchronization loop ensures stable execution:
planner → trajectory → controller → feedback → planner

---

## Control Strategy

A velocity-based control law is implemented:
v = x_dot_desired + Kp * (x_desired - x_current)

Joint velocities are computed using a damped pseudo-inverse Jacobian:
J^+ = J^T (J J^T + λI)^(-1)

This ensures:

- Stability near singularities  
- Smooth trajectory tracking  

---

## Synchronization Mechanism

- Controller publishes `/controller_ready`  
- Planner sends next trajectory point only when ready  

This prevents:

- Skipped trajectory points  
- Timing mismatch  
- Unstable motion  

---

## Results

### Tracking Performance

- Mean Error: ~0.0141 m  
- Max Error: ~0.0192 m  

### Observations

- Strong alignment between planned and executed trajectories  
- Smooth and stable motion  
- Minor deviations due to discretization and control limits  

---

## Engineering Significance

Sprint 7 establishes a complete robotics execution pipeline:

- MATLAB → Trajectory Optimization  
- ROS2 → Real-Time Control  
- Feedback → Error Correction  
- Logging → Performance Validation  

---

## Conclusion

Sprint 7 completes the transition from:

**Trajectory Optimization → Real-Time Robotic Execution**

Project AURA now achieves:

- End-to-end motion pipeline  
- Real-time trajectory tracking  
- Quantitative validation of performance  
- ROS2-based modular control architecture  

---

## 🚀 Future Work (Bonus Sprint)

A potential **Sprint 8** can extend this work to:

- Full UR5 simulation in Gazebo  
- Physics-based validation  
- Sensor integration  
- Digital twin development  

---

## Project Status

Version: v7.0  

Sprint 1 – Kinematic Modeling – Complete  
Sprint 2 – Differential Kinematics & Robust Control – Complete  
Sprint 3 – Inverse Dynamics & Energy Optimization – Complete  
Sprint 4 – Payload-Aware Task-Level Energy Analysis – Complete  
Sprint 5 – Multi-Trajectory Tracking & Energy–Dexterity Analysis – Complete  
Sprint 6 – Trajectory Optimization with Adaptive Sampling – Complete  
Sprint 7 – ROS2-Based Trajectory Execution & Validation – Complete  

🎯 **Project AURA Core Development Completed**

🚀 *Future Extension:*  
Sprint 8 – Gazebo-Based Simulation & Digital Twin (Planned)

---

## Author

**Nihal Sanjay Seth**