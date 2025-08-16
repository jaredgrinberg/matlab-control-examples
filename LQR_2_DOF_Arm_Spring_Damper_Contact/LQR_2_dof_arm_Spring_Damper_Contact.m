% LQR Controller for 2-DOF Arm with Ground Contact
clear all;
close all;
clc;

% Add parent directory to path for CasADi and Spatial_v2
addpath(genpath('..'))
run ../setup.m
import casadi.*

%% Initialize robot and parameters
robot = model_2dofArm();
p.l = [0.5; 1]; % Link lengths
p.r = [0.5; 0.5]; % COM distances
p.M = [0.5; 1]; % Link masses
p.J = [1e-3; 2e-3]; % Link inertias

p.params = [p.l; p.r; p.M; p.J];

%% Simulation parameters
dt = 0.01; % Control frequency
tDuration = 2; % Simulation duration
MAXITER = floor(tDuration/dt);

% State: X = [q1, q2, dq1, dq2, u]
X0 = [pi/4; 0; 0; 0; 0]; % Initial state with contact state u
XDes = [-pi/12; -pi/4; 0; 0; 0]; % Desired state with contact state
Unom = [0; 0]; % Nominal control input

%% LQR Controller Design
% Get linearized system matrices (ignore contact state for control)
[A, B] = get_linearized_dynamics(XDes(1:4), Unom, p, dt);

% LQR weights
Q = diag([10 10 1 1]); % State cost
R = diag([0.1 0.1]); % Control cost

% Compute LQR gains
[K, S, P] = dlqr(A, B, Q, R);

%% Simulation
[tout, Xout, Uout] = deal([]);
F_contact = zeros(MAXITER, 2);  % Pre-allocate F_contact
X = X0;
t = 0;
contact_idx = 1;

for ii = 1:MAXITER
    % Compute control input (only using position and velocity states)
    U = -K * (X(1:4) - XDes(1:4));
    
    % Simulate one step with contact
    [t_temp, X_temp] = ode45(@(t,X)arm_dynamics_with_contact(t, X, U, p), [t t+dt], X);
    
    % Compute contact force for logging
    [p_ee, v_ee, J_ee, ~, ~] = fcn_FK_2_link(X_temp(end,1:2)', X_temp(end,3:4)', p.params);
    Jc = J_ee;
    z_obs = -1; % Ground height
    pos = p_ee - [0; z_obs];
    vel = v_ee;
    [Fc, ~] = contactDynamics(pos, vel, X_temp(end,5));
    
    % Store results
    tout = [tout; t_temp];
    Xout = [Xout; X_temp];
    Uout = [Uout; repmat(U', size(t_temp))];
    F_contact(contact_idx,:) = Fc';
    contact_idx = contact_idx + 1;
    
    % Update state
    X = X_temp(end,:)';
    t = t + dt;
end

%% Plot results
figure(1);
subplot(2,2,1);
plot(tout, Xout(:,1:2));
title('Joint Angles');
legend('q1', 'q2');
ylabel('rad');

subplot(2,2,2);
plot(tout, Xout(:,3:4));
title('Joint Velocities');
legend('dq1', 'dq2');
ylabel('rad/s');

subplot(2,2,3);
plot(tout, Uout);
title('Control Inputs');
legend('u1', 'u2');
ylabel('Nm');

subplot(2,2,4);
t_contact = linspace(0, tDuration, MAXITER);
plot(t_contact, F_contact);
title('Contact Forces');
legend('F_x', 'F_y');
ylabel('N');

%% Visualize motion
figure(2);
showmotion(robot, tout, Xout(:,1:2)');

%% Helper Functions
function dXdt = arm_dynamics_with_contact(t, X, U, p)
    % Extract states
    q = X(1:2);
    dq = X(3:4);
    u = X(5); % Contact state
    
    % Get kinematics
    [p_ee, v_ee, J_ee, ~, ~] = fcn_FK_2_link(q, dq, p.params);
    Jc = J_ee;
    
    % Compute contact force
    z_obs = -1; % Ground height
    pos = p_ee - [0; z_obs];
    vel = v_ee;
    [Fc, du] = contactDynamics(pos, vel, u);
    
    % Get dynamics matrices
    [H, C, G] = fcn_dynamics_2_link(q, dq, p.params);
    
    % Compute accelerations including contact forces
    ddq = H \ (U - C*dq - G + Jc' * Fc);
    
    % Return state derivatives
    dXdt = [dq; ddq; du];
end

function [f, du] = contactDynamics(p, v, u)
    % Spring-damper parameters
    K = 1e3;  % Spring constant 
    D = 50;  % Damping constant
    mu = 0.3; % Friction coefficient
    
    z = p(2);  % Vertical position
    dz = v(2); % Vertical velocity
    
    % Normal force calculation
    zr = sqrt(max(0,-z));
    fn = zr * ((-K)*z - D*dz);
    fn = max(0, fn);
    
    % Friction force calculation
    if mu == 0 || fn == 0
        f = [0; fn];
        du = 0;
        return
    end
    
    dx = v(1);
    du = (-K/D) * u;
    
    % Tangential force
    ft = -K * zr * u - D * zr * dx;
    
    % Apply friction cone constraint
    ft_max = mu * fn;
    if abs(ft) > ft_max
        ft = sign(ft) * ft_max;
        du = -(ft + K * zr * u) / (D * zr);
    end
    
    f = [ft; fn];
end

function [A, B] = get_linearized_dynamics(X, U, p, dt)
    % Numerical linearization around equilibrium
    epsilon = 1e-6;
    n = length(X);
    m = length(U);
    
    % Initialize matrices
    Ac = zeros(n,n);
    Bc = zeros(n,m);
    
    % Compute nominal dynamics
    f0 = arm_dynamics_with_contact(0, [X; 0], U, p);
    f0 = f0(1:n); % Ignore contact state for control
    
    % Compute A matrix
    for i = 1:n
        X_perturbed = X;
        X_perturbed(i) = X_perturbed(i) + epsilon;
        f_perturbed = arm_dynamics_with_contact(0, [X_perturbed; 0], U, p);
        f_perturbed = f_perturbed(1:n); % Ignore contact state for control
        Ac(:,i) = (f_perturbed - f0) / epsilon;
    end
    
    % Compute B matrix
    for i = 1:m
        U_perturbed = U;
        U_perturbed(i) = U_perturbed(i) + epsilon;
        f_perturbed = arm_dynamics_with_contact(0, [X; 0], U_perturbed, p);
        f_perturbed = f_perturbed(1:n); % Ignore contact state for control
        Bc(:,i) = (f_perturbed - f0) / epsilon;
    end
    
    % Convert to discrete time
    A = eye(n) + dt * Ac;
    B = dt * Bc;
end