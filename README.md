# MATLAB Control Examples

Collection of MATLAB control theory examples including LQR controllers, trajectory optimization, and robotics simulations.

## Projects

### LQR Controllers
- **[Cart_Pole_LQR](Cart_Pole_LQR/)** - Linear Quadratic Regulator for cart-pole stabilization
- **[LQR_2_DOF_Arm](LQR_2_DOF_Arm/)** - LQR control of 2-link robotic arm
- **[LQR_2_DOF_Arm_Spring_Damper_Contact](LQR_2_DOF_Arm_Spring_Damper_Contact/)** - LQR control with ground contact dynamics
- **[Quadrotor_LQR](Quadrotor_LQR/)** - Planar quadrotor stabilization using LQR

### Trajectory Optimization
- **[TrajOpt_Cart_Pole](TrajOpt_Cart_Pole/)** - Swing-up control using direct collocation
- **[TrajOpt_2_DOF_Cart_Pole](TrajOpt_2_DOF_Cart_Pole/)** - Multi-link cart pole trajectory optimization

## Dependencies

### Required
- MATLAB with Symbolic Math Toolbox

### For Trajectory Optimization Projects
**CasADi** - Download from [https://web.casadi.org/get/](https://web.casadi.org/get/)
- Choose your platform (Windows, Linux, or macOS)
- Extract to the root directory of this repo

**Spatial_v2_extended** - Download from [https://github.com/ROAM-Lab-ND/spatial_v2_extended](https://github.com/ROAM-Lab-ND/spatial_v2_extended)
- Clone or download ZIP
- Place in the root directory of this repo

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/jaredgrinberg/matlab-control-examples.git
   cd matlab-control-examples
   ```

2. Download dependencies:
   - **CasADi**: Download for your platform and extract here
   - **Spatial_v2**: Clone the repo or download ZIP and place here

3. Update `setup.m` with correct CasADi folder name:
   ```matlab
   % In setup.m, update the path to match your CasADi version
   addpath(genpath('casadi-YOUR-VERSION-HERE'))
   addpath(genpath('spatial_v2_extended'))
   ```

4. Run any example:
   ```matlab
   cd Cart_Pole_LQR
   MAIN_cart_pole_dlqr
   ```

## Project Structure

```
matlab-control-examples/
├── Cart_Pole_LQR/                    # LQR cart-pole control
├── LQR_2_DOF_Arm/                    # LQR robotic arm control
├── LQR_2_DOF_Arm_Spring_Damper_Contact/  # LQR with contact dynamics
├── Quadrotor_LQR/                    # LQR quadrotor control
├── TrajOpt_Cart_Pole/                # Trajectory optimization examples
├── TrajOpt_2_DOF_Cart_Pole/          
├── casadi-YOUR-VERSION/              # Download and place here
├── spatial_v2_extended/              # Clone and place here
├── setup.m                          # Dependency initialization
└── README.md
```

## Usage

### LQR Examples
Most LQR examples run independently:
```matlab
cd LQR_2_DOF_Arm
symbolic_kinematics_dynamics_generator_N_link  % Generate dynamics
LQR_2_dof_arm  % Run controller
```

### Trajectory Optimization Examples
Require CasADi initialization:
```matlab
cd TrajOpt_Cart_Pole
run ../setup.m
import casadi.*
MAIN_cartpole
```

## Theory

These examples demonstrate:
- **LQR Theory** - Optimal linear control for stabilization around equilibrium
- **Trajectory Optimization** - Direct collocation methods for optimal control
- **Contact Dynamics** - Spring-damper models for ground interaction
- **Robotics Kinematics** - Forward kinematics and Jacobian computation
- **System Dynamics** - Mass-spring-damper and rigid body dynamics

## Platform Support

After downloading CasADi for your platform, update the path in `setup.m`:
- **Windows** - `casadi-windows64-matlabXXXX`
- **Linux** - `casadi-linux64-matlabXXXX` 
- **macOS** - `casadi-osxXX-matlabXXXX`

The `setup.m` file adds the appropriate CasADi version to your MATLAB path.
