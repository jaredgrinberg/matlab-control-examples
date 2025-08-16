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
- [Robotics Toolbox](https://github.com/petercorke/robotics-toolbox-matlab) (for visualization)

### Included
- **CasADi** - Multiple platform versions included (Linux, Windows, macOS)
- **Spatial_v2_extended** - Spatial vector algebra library

## Quick Start

1. Clone this repository:
   ```bash
   git clone https://github.com/jaredgrinberg/matlab-control-examples.git
   ```

2. Open MATLAB and navigate to the repo directory

3. Run any example:
   ```matlab
   cd Cart_Pole_LQR
   MAIN_cart_pole_dlqr
   ```

   For trajectory optimization examples:
   ```matlab
   cd TrajOpt_Cart_Pole
   run ../setup.m  % Initialize CasADi
   MAIN_cartpole
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
├── casadi-3.6.4-linux64-matlab2018b/     # CasADi for Linux
├── casadi-3.6.4-windows64-matlab2018b/   # CasADi for Windows
├── casadi_3.6.7_osx_arm64/               # CasADi for macOS
├── spatial_v2_extended/              # Spatial vector library
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

- **Windows** - Use `casadi-3.6.4-windows64-matlab2018b`
- **Linux** - Use `casadi-3.6.4-linux64-matlab2018b`
- **macOS** - Use `casadi_3.6.7_osx_arm64`

The `setup.m` file automatically detects your platform and adds the appropriate CasADi version to your path.

## License

MIT License
