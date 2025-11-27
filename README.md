# **Flywheel Energy Storage System – MATLAB Model**

### *Project 3 – ME 4053 (Fall 2025)*

Modular simulation environment for analyzing the electromagnetic flywheel storage system, including rotor physics, thermal behavior, cycle efficiency, and magnetic bearing dynamics.

---

## Folder Structure Overview

```
project3/
│
├── main.m
│
├── utils/
│   ├── params.m
│   └── (misc utilities)
│
├── flywheel/
│   ├── compute_geometry.m
│   ├── compute_inertia.m
│   ├── compute_temperature.m
│   ├── compute_losses.m
│   ├── compute_soc.m
│   ├── calcRatedTorque.m
│   └── (flywheel-specific helpers)
│
├── storage/
│   ├── load_cycle.m
│   ├── simulate_cycle.m
│   ├── deliverable1a.m
│   ├── deliverable1b.m
│   └── (design sweep files added later)
│
└── amb/
    ├── amb_ode.m
    ├── simulate_step.m
    ├── compute_dynamic_stiffness.m
    ├── compute_runout.m
    └── (controller design added later)
```



## What Each Folder Does

### **`utils/` — Global Parameters & Helpers**

Contains project-wide constants and simple utility functions.

* **params.m** – Loads all baseline values from Table 1 & Appendix A
  (densities, clearances, emissivities, max speed, temp limits, etc.)
* Small shared helper functions go here.

---

### **`flywheel/` — Rotor Physics**

Implements all physical models for the rotor, magnets, shaft, losses, and thermal radiation.

* **compute_geometry.m**
  Build full rotor + motor + housing geometry (areas, radii, lengths, view factor).
* **compute_inertia.m**
  Compute masses + polar inertia of flywheel, shaft, magnets.
* **compute_temperature.m**
  Radiative thermal model (gray-body rotor–housing heat transfer).
* **compute_losses.m**
  Wrapper for EE `.p` functions → rotorLosses & statorLosses.
* **compute_soc.m**
  Convert between state-of-charge and rotational speed.
* **calcRatedTorque.m**
  Rated torque from magnetic shear (I_pu = 1.0).

---

### **`storage/` — Storage Cycle Simulation**

Implements all time-dependent system behavior (ω(t), SoC(t), efficiency).

* **load_cycle.m**
  Loads baseline or team-specific power command profile.
* **simulate_cycle.m**
  Integrates rotor ODE with real losses, torque limits, and SoC mapping.
* **deliverable1a.m**
  Losses + rotor temperature vs State of Charge.
* **deliverable1b.m**
  Specific energy and specific power of baseline design.

---

### **`amb/` — Active Magnetic Bearing Model**

Implements dynamics and control of the radial magnetic bearings.

* **amb_ode.m**
  Full cascaded current + position loop dynamics (Appendix B structure).
* **simulate_step.m**
  Step disturbance response at 0% and 100% SoC.
* **compute_dynamic_stiffness.m**
  Closed-loop stiffness vs excitation frequency.
* **compute_runout.m**
  Rotor orbit amplitude vs SoC for mass imbalance.

---

##  `main.m` — Project Driver

Runs the overall workflow

