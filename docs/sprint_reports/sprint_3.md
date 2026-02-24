# Sprint 3 – Inverse Dynamics & Energy Analysis

## Objective

Extend the UR5 kinematic framework into a full dynamic analysis environment to evaluate torque demand, mechanical power, and energy efficiency during circular Cartesian motion.

This sprint shifts the project from purely kinematic control to physics-based actuation analysis.

---

## What Was Implemented

### 1. Full Inverse Dynamics Model

Using official UR5 parameters:

- Mass matrix M(q)
- Coriolis matrix C(q, q̇)
- Gravity vector G(q)

Torque model:

τ = M(q)q̈ + C(q,q̇)q̇ + G(q)

Acceleration computed numerically from joint velocities.

---

### 2. Circular Motion Energy Study

Robot commanded to follow circular trajectories of varying radii:

r ∈ {0.03, 0.1, 0.2, 0.3} m

For each radius:

- Total mechanical energy
- RMS joint torque norm
- RMS mechanical power
- Energy per meter traveled
- RMS Cartesian tracking error

Computed for:

- Fixed Damping
- Adaptive Manipulability-Based Damping

---

### 3. Dynamic Performance Comparison

Key findings:

- Torque demand increases with radius
- Power consumption increases nonlinearly with radius
- Energy per meter reveals efficiency trends
- Adaptive damping improves tracking accuracy at larger radii
- Adaptive control is beneficial near low-manipulability regions
- Fixed damping can be more energy-efficient in well-conditioned zones

---

## Detailed Dynamic Case (r = 0.2 m)

Inverse dynamics visualization includes:

- Joint torque norm over time
- Mechanical power profile
- Individual joint torques

Observations:

- Shoulder joint (τ₂) dominates gravitational load
- Power sign change indicates regenerative phases
- Torque spikes occur at curvature transitions

---

## Engineering Insight

This sprint demonstrates that:

Energy consumption is not only a function of path length,
but strongly influenced by:

- Jacobian conditioning
- Damping strategy
- Required accelerations
- Dynamic coupling effects

This establishes a foundation for actuation optimization.

---

## Status

Sprint 3 Complete  
Release: v3.0