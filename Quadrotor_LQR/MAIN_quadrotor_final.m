% Planar Quadrotor LQR Controller
% Dynamics: https://underactuated.mit.edu/acrobot.html
clear all;
close all;
clc;

%% Quadrotor physical parameters
p.mass = 1;         % Quadrotor mass (kg)
p.inertia = 1e-3;   % Rotational inertia (kg*m^2)
p.halfWidth = 0.2;  % Half distance between rotors (m)
p.g = 9.81;         % Gravitational acceleration (m/s^2)

%% Simulation parameters
tDuration = 1;      % Total simulation time (s)
dt= 0.01;           % Control frequency timestep (100hz)
tstart = 0;
tend = dt;
MAXITER = floor(tDuration / dt);

%% Initial and desired states
% State vector: X = [x, y, th, dx, dy, dth]
X0 = [0 1 0.5 -1 -2 0]';  % Initial state [position, angle, velocities]
XDes = [0 1 0 0 0 0]';     % Desired state (hover at x=0, y=1)
Unom = 0.5 * p.mass * p.g * [1,1]; % Nominal control (hover thrust)

%% LQR Controller Design
[Ac, Bc] = get_Jacobian(XDes, Unom, p); % Linearize dynamics around equilibrium

% Convert continuous to discrete time
A = eye(6) + dt * Ac;
B = dt * Bc;

% LQR cost matrices
Q = diag([10 10 1 0.01 0.01 0.1]); % State cost weights
R = diag(0.001 * [1 1]);           % Control cost weights

% Compute LQR gains
[K, S, P] = dlqr(A, B, Q, R);

%% Simulation loop
[tout, Xout, Uout] = deal([]);
for ii= 1:MAXITER
    % LQR controller
    U = K * (XDes - X0);
    
    % Simulate one timestep
    [t, X] = ode45(@(t,X)dynamics(t, X, U, p), [tstart, tend], X0);
    
    % Update for next iteration
    X0 = X(end, :)';
    tstart = tend;
    tend = tstart + dt;
    
    % Store results
    tout = [tout; t];
    Xout = [Xout; X];
    nt = length(t);
    Uout = [Uout; repmat(U(:)', [nt,1])];
end

%% Plot results
figure(1);
subplot(2,3,1);
plot(tout, Xout(:,1));
title('X Position');
xlabel('Time (s)');
ylabel('x (m)');
grid on;

subplot(2,3,2);
plot(tout, Xout(:,2));
title('Y Position');
xlabel('Time (s)');
ylabel('y (m)');
grid on;

subplot(2,3,3);
plot(tout, Xout(:,3));
title('Angle');
xlabel('Time (s)');
ylabel('\theta (rad)');
grid on;

subplot(2,3,4);
plot(tout, Xout(:,4));
title('X Velocity');
xlabel('Time (s)');
ylabel('dx (m/s)');
grid on;

subplot(2,3,5);
plot(tout, Xout(:,5));
title('Y Velocity');
xlabel('Time (s)');
ylabel('dy (m/s)');
grid on;

subplot(2,3,6);
plot(tout, Xout(:,6));
title('Angular Velocity');
xlabel('Time (s)');
ylabel('d\theta (rad/s)');
grid on;

% Control inputs
figure(2);
subplot(2,1,1);
plot(tout, Uout(:,1));
title('Left Rotor Thrust');
xlabel('Time (s)');
ylabel('u1 (N)');
grid on;

subplot(2,1,2);
plot(tout, Uout(:,2));
title('Right Rotor Thrust');
xlabel('Time (s)');
ylabel('u2 (N)');
grid on;

% Trajectory plot
figure(3);
plot(Xout(:,1), Xout(:,2), 'b-', 'LineWidth', 2);
hold on;
plot(Xout(1,1), Xout(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot(XDes(1), XDes(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Target
plot([-2, 2], [0, 0], 'k-', 'LineWidth', 2); % Ground
title('Quadrotor Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Trajectory', 'Start', 'Target', 'Ground', 'Location', 'best');
grid on;
axis equal;

%% Animation
figure(4);
nt = length(tout);
for ii = 1:100:nt
    clf
    hold on;
    axis equal;
    xlim([-1, 1]);
    ylim([-0.1 2]);
    
    % Draw quadrotor at current state
    plotQuadrotor(tout(ii), Xout(ii,:), Uout(ii,:), p);
    
    % Draw trajectory trace
    plot(Xout(1:ii, 1), Xout(1:ii, 2), 'k');
    
    % Draw ground
    plot([-10, 10], [0,0], 'k');
    
    title(sprintf('Quadrotor Animation - Time: %.2f s', tout(ii)));
    
    pause(0.001);
end

%% Dynamics Function
function dXdt = dynamics(t, X, U, p)
    % Extract state variables
    [x, y, th, dx, dy, dth] = deal(X(1), X(2), X(3), X(4), X(5), X(6));
    [u1, u2] = deal(U(1), U(2));
    
    % Quadrotor dynamics equations
    ddx = -1/p.mass * (u1 + u2) * sin(th);              % x acceleration
    ddy = 1/p.mass * (u1 + u2) * cos(th) - p.g;         % y acceleration  
    ddth = 1/p.inertia * p.halfWidth * (u1 - u2);       % angular acceleration
    
    % Return state derivatives
    dXdt = [dx dy dth ddx ddy ddth]';
end

%% Quadrotor visualization
function plotQuadrotor(t, X, U, p)
    % Extract state and control
    [x, y, th, dx, dy, dth] = deal(X(1), X(2), X(3), X(4), X(5), X(6));
    [u1, u2] = deal(U(1), U(2));
    r = p.halfWidth;
    
    % Quadrotor body endpoints
    p1 = [x; y] + rot(th) * [r; 0];   % Right rotor position
    p2 = [x; y] + rot(th) * [-r; 0];  % Left rotor position
    chain_body = [p1, p2];
    
    % Thrust force visualization
    scalingFactor = 0.05;
    p1u = p1 + rot(th) * [0; u1] * scalingFactor; % Right thrust vector
    p2u = p2 + rot(th) * [0; u2] * scalingFactor; % Left thrust vector
    chain_u1 = [p1, p1u];
    chain_u2 = [p2, p2u];
    
    % Draw quadrotor
    hold on
    axis equal;
    plot(chain_body(1,:), chain_body(2,:), 'k','LineWidth', 5); % Body
    plot(chain_u1(1,:), chain_u1(2,:), 'r');                   % Right thrust
    plot(chain_u2(1,:), chain_u2(2,:), 'r');                   % Left thrust
end

%% 2D rotation matrix
function R = rot(th)
    c = cos(th);
    s = sin(th);
    R = [c, -s; s, c];
end

%% Linearized dynamics matrices
function [Ac, Bc] = get_Jacobian(X, U, p)
    % Extract equilibrium state and control
    [x, y, th, dx, dy, dth] = deal(X(1), X(2), X(3), X(4), X(5), X(6));
    [u1, u2] = deal(U(1), U(2));
    
    % System parameters
    r = p.halfWidth;
    g = p.g;
    mass = p.mass;
    inertia = p.inertia;
    
    % State matrix A: dx/dt = A*x + B*u
    Ac = zeros(6);
    Ac(1:3, 4:6) = eye(3);                        % Position derivatives = velocities
    Ac(4,3) = -(cos(th)*(u1 + u2))/mass;          % ddx dependence on theta
    Ac(5,3) = -(sin(th)*(u1 + u2))/mass;          % ddy dependence on theta
    
    % Input matrix B
    Bc = zeros(6,2);
    Bc(4,:) = [-sin(th)/mass, -sin(th)/mass];     % ddx control coupling
    Bc(5,:) = [cos(th)/mass, cos(th)/mass];       % ddy control coupling  
    Bc(6,:) = [r/inertia, -r/inertia];            % ddth control coupling
end