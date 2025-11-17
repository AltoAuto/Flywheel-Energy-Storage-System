# **Project 3: Flywheel Energy Storage System**

This repository contains all MATLAB code needed to:

1. Evaluate the **baseline flywheel system**
2. Run the **storage cycle simulation**
3. Compute **losses, temperature, SOC, inertia**
4. Build and simulate **AMB plant + controller**
5. Perform **design sweeps** (magnet thickness, max speed)
6. Compare **baseline vs optimal design**

Everything is modular and each folder has one job.

---

# **Folder Structure**

```
project3/
│   main.m
│
├── flywheel/
│   compute_inertia.m          % mass, inertia, tip-speed checks
│   compute_soc.m              % SOC omega mapping
│   compute_losses.m           % wraps rotorLosses, statorLosses, shear
│   compute_temperature.m      % thermal ODE model
│
├── storage/
│   load_cycle.m               % loads baseline/new cycle
│   simulate_cycle.m           % runs the cycle: ω(t), SOC(t), T(t), efficiency
│
├── amb/
│   build_amb_tf.m             % builds AMB plant model (TF or SS)
│   simulate_step.m            % step disturbance response
│   compute_dynamic_stiffness.m% Kdyn(freq)
│   compute_runout.m           % imbalance → orbit/runout vs SOC
│
└── utils/
    params.m                   % all physical parameters + design settings
    plots.m                    % plotting helpers
```

---

# **Purpose of Each Folder**

### **flywheel/**

Physics of the rotating group.

* Inertia + mass
* SOC formula
* Loss calculations
* Thermal model (steady-state or dynamic)

### **storage/**

Time-domain power cycling.

* Loads cycle data
* Integrates rotational dynamics
* Computes efficiency + self-discharge recovery

### **amb/**

Active Magnetic Bearing modeling.

* Build plant
* Apply controller
* Step disturbance
* Dynamic stiffness
* Runout simulation

### **utils/**

Shared utilities.

* All parameters in one place
* Plot formatting

---

# **How to Use**

1. Edit **utils/params.m** to set baseline or design parameters.
2. Run **main.m**.
3. All figures and results will be generated automatically.

---

# **Output**

Running `main.m` should produce:

* Losses vs SOC
* Temperature vs SOC
* Specific energy & specific power
* Storage-cycle efficiency
* AMB step response
* Dynamic stiffness
* Runout vs SOC
* Design sweep figures
* Comparison plots

---

If you want, I can also generate a **ROADMAP.md** (what to implement first, second, etc.).
