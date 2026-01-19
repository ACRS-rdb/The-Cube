# The-Cube — Reaction Wheel Cube Simulation (MuJoCo + Drake MPC)

A high-fidelity simulation of a reaction wheel-based inverted pendulum cube controlled by real-time linear Model Predictive Control (MPC). Physics simulation runs in **MuJoCo**, and the quadratic program is solved at each control step using **Drake** optimization tools.

## Features

- MuJoCo multibody dynamics simulation with contact and collision
- Online linear MPC (QP solved per control step)
- Real-time system linearization using MuJoCo's finite-difference perturbation method
- Designed to support future transfer to real hardware implementation

## Prerequisites

Install the following Python packages:

```bash
pip install mujoco pydrake numpy scipy
```

## Repository Structure

```text
.
├── simulation/
│   └── mujoco_simulation.py        # Main simulation and MPC controller
└── The_Cube_description/
    ├── The_Cube.xml                # MuJoCo MJCF model definition
    └── meshes/                     # Modified STL/mesh assets
```

## Usage

Run the simulation from the repository root:

```bash
python simulation/mujoco_simulation.py
```

A MuJoCo viewer window will open, displaying the cube balancing using the computed control torques.

---

## Credits & Licensing

### 3D Mechanical Design

This project uses and modifies the self-balancing cube design by **remrc**:

- **Original design**: [Self balancing cube (cubli) by remrc](https://www.thingiverse.com/thing:6695891)
- **License**: [CC BY-SA 3.0](https://creativecommons.org/licenses/by-sa/3.0/)
- 
**Nidec 24H BLDC Motor CAD by VoltLAB ([GrabCAD](https://grabcad.com/library/nidec-24h-6mm-bldc-brushless-motor-with-encoder-2gt-pulley-servo-1))**
**MPU6050 CAD by Siddharaj Junnarkar ([GrabCAD](https://grabcad.com/library/mpu6050-8))**

**Modifications made**: The original STL/mesh files were modified for MuJoCo simulation compatibility (geometry cleanup, scaling, coordinate frame adjustments) and to support future real-world hardware builds.

**ShareAlike requirement**: If you redistribute modified 3D model files derived from this design, you must release them under the same CC BY-SA 3.0 (or compatible) license and provide appropriate attribution to remrc.

### Source Code

The source code in this repository is licensed under the terms specified in the `LICENSE` file.

### Third-Party Dependencies

- **MuJoCo**: Apache License 2.0
- **Drake / pydrake**: BSD 3-Clause License

---

## Author
  
National Taipei University of Technology (NTUT)  
Automatic Control Research Society (ACRS)

---

## Theoretical Background

The control system implements linear MPC based on a discrete-time state-space model derived from the nonlinear multibody dynamics. System linearization around the upright equilibrium point is performed in real-time using **MuJoCo's built-in finite-difference perturbation method**, which computes the Jacobian matrices by numerically perturbing the state and control variables.

> **Note**: Detailed mathematical derivations, including state-space formulation and cost function design, will be documented in an upcoming technical blog post.
