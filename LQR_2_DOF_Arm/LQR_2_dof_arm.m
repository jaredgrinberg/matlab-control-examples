% LQR Controller for 2-DOF Arm - Using Custom Dynamics
clear all;
close all;
clc;

% Add parent directory to path for CasADi and Spatial_v2
addpath(genpath('..'))
run ../setup.m
import casadi.*

%% Initialize robot and parameters
robot = model_2dofArm();
p.l = [0.5; 1]; % Link lengths (column vector)
p.r = [0.5; 0.5]; % COM distances relative to link length
p.M = [0.5; 1]; % Link masses (column vector)
p.J = [1e-3; 2e-3]; % Link inertias (column vector)

% Generate your dynamics functions
% First run your dynamics generation script to create the function files
% This will create: fcn_dynamics_2_link.m

% Create parameters vector as expected by your generated function
p.params = [p.l; p.r; p.M; p.J]; % Combined parameter vector

%% Simulation parameters
dt = 0.01; % Control frequency
tDuration = 2; % Simulation duration
MAXITER = floor(tDuration/dt); % Number of simulation steps

% State: X = [q1, q2, dq1, dq2]
X0 = [pi/4; 0; -6; -2]; % Initial state
XDes = [pi/2; 0; 0; 0]; % Desired state
Unom = [0; 0]; % Nominal control input (zero torque)

%% LQR Controller Design
% Get linearized system matrices
[A, B] = get_linearized_dynamics(XDes, Unom, p, dt);

% LQR weights
Q = diag([10 10 1 1]); % State cost
R = diag([0.1 0.1]); % Control cost

% Compute LQR gains
[K, S, P] = dlqr(A, B, Q, R);

%% Simulation
[tout, Xout, Uout] = deal([]);
X = X0;
t = 0;

for ii = 1:MAXITER
    % Compute control input
    U = K * (XDes - X);
    
    % Simulate one step
    [t_temp, X_temp] = ode45(@(t,X)arm_dynamics(t, X, U, p), [t t+dt], X);
    
    % Store results
    tout = [tout; t_temp];
    Xout = [Xout; X_temp];
    Uout = [Uout; repmat(U', size(t_temp))];
    
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

%% Visualize motion
figure(2);
showmotion(robot, tout, Xout(:,1:2)');

%% Helper Functions
function dXdt = arm_dynamics(t, X, U, p)
    % Extract states
    q = X(1:2);
    dq = X(3:4);
    
    % Get dynamics matrices using your generated function
    [H, C, G] = fcn_dynamics_2_link(q, dq, p.params);
    
    % Compute accelerations
    % Equation of motion: H(q)ddq + C(q,dq)dq + G(q) = tau
    % Solve for ddq: ddq = H^(-1) * (tau - C*dq - G)
    ddq = H \ (U - C*dq - G);
    
    % Return state derivatives
    dXdt = [dq; ddq];
end

function [A, B] = get_linearized_dynamics(X, U, p, dt)
    % Numerical linearization around equilibrium
    epsilon = 1e-6; % Small perturbation for numerical derivatives
    n = length(X); % Number of states
    m = length(U); % Number of inputs
    
    % Initialize matrices
    Ac = zeros(n,n);
    Bc = zeros(n,m);
    
    % Compute nominal dynamics
    f0 = arm_dynamics(0, X, U, p);
    
    % Compute A matrix using finite differences
    for i = 1:n
        X_perturbed = X;
        X_perturbed(i) = X_perturbed(i) + epsilon;
        f_perturbed = arm_dynamics(0, X_perturbed, U, p);
        Ac(:,i) = (f_perturbed - f0) / epsilon;
    end
    
    % Compute B matrix using finite differences
    for i = 1:m
        U_perturbed = U;
        U_perturbed(i) = U_perturbed(i) + epsilon;
        f_perturbed = arm_dynamics(0, X, U_perturbed, p);
        Bc(:,i) = (f_perturbed - f0) / epsilon;
    end
    
    % Convert to discrete time
    A = eye(n) + dt * Ac;
    B = dt * Bc;
end