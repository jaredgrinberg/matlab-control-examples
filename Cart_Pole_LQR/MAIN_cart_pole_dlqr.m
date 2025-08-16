% Cart Pole DLQR (Discrete-time Linear Quadratic Regulator)

clear all;
close all;
clc;

%% System Parameters
p.m_c = 1.0;      % mass of cart (kg)
p.m_p = 0.1;      % mass of pendulum (kg)
p.l = 0.5;        % length of pendulum (m)
p.g = 9.81;       % gravity (m/s^2)

tDuration = 5;    % Total simulation time (seconds)
dt = 0.01;        % Time step (seconds) - 100Hz control frequency
tstart = 0;       % Initial time for each integration step
tend = dt;        % End time for each integration step
MAXITER = floor(tDuration / dt);  % Number of simulation steps

% X0 = [x, th, dx, dth]
X0 = [0, 2, 0, 0]';   % Initial state
XDes = [0, pi, 0, 0]';  % Desired state

%% LQR Controller
% Get linearized system matrices around the desired state
[A, B] = get_cart_pole_dynamics(XDes, dt, p);

Q = diag([1, 10, 1, 1]); % Gain on state, higher on theta
R = 0.01; % Penalization on control; smaller valuer more aggressive control

% Compute optimal feedback gain matrix K
[K, S, P] = dlqr(A, B, Q, R);


%% Simuation
[tout, Xout, Uout] = deal([]);
X = X0;

for ii= 1:MAXITER
    % Compute control input using LQR feedback
    U = K * (XDes - X);

    % Simulator
    [t, X_temp] = ode45(@(t,X)cart_pole_dynamics(t, X, U, p), [tstart, tend], X);

    % Update Initial Condition for Next Iteration
    X = X_temp(end, :)';
    tstart = tend;
    tend = tstart + dt;

    tout = [tout; t];
    Xout = [Xout; X_temp];
    Uout = [Uout; U*ones(length(t), 1)];

end

% nexttile
% plot(t, X(:, 1))
% nexttile
% plot(t, X(:, 2))
% nexttile
% plot(t, X(:, 3))

%% Animation
nt = length(tout);
for ii = 1:100:nt % Animate every 100th frame for smoother visual
    clf
    hold on;
    xlim([-4, 4]);
    ylim([-1 1]);
    plot_cart_pole(Xout(ii,:), p);
    drawnow;
    pause(0.01);
end
%% Dynamics Function
function dX = cart_pole_dynamics(t, X, u, p)
    % Extracting states
    x = X(1);        % Cart position
    theta = X(2);    % Pendulum angle
    dx = X(3);       % Cart velocity
    dtheta = X(4);   % Pendulum angular velocity
    
    % System parameters
    mc = p.m_c;      % Cart mass
    mp = p.m_p;      % Pendulum mass
    l = p.l;         % Pendulum length
    g = p.g;         % Gravity
    fx = u;          % Control force
    
    % X acceleration
    ddx = 1/(mc + mp*sin(theta)^2) * (fx + mp*sin(theta)*(l*dtheta^2 + g*cos(theta)));
    
    % Theta acceleration
    ddtheta = 1/(l*(mc + mp*sin(theta)^2)) * (-fx*cos(theta) - mp*l*dtheta^2*cos(theta)*sin(theta) - (mc + mp)*g*sin(theta));
    
    % State derivative
    dX = [dx; dtheta; ddx; ddtheta];
end

function [A, B] = get_cart_pole_dynamics(X, dt, p)
    % Numerical computation of Jacobians
    eps = 1e-5;     % Small perturbation for numerical derivatives
    n = length(X);  % Number of states
    
    % Initialize matrices
    Ac = zeros(n,n);
    Bc = zeros(n,1);
    
    % Compute A matrix using finite difference
    for i = 1:n
        X_temp = X;
        X_temp(i) = X_temp(i) + eps;
        f1 = cart_pole_dynamics(0, X_temp, 0, p);
        f2 = cart_pole_dynamics(0, X, 0, p);
        Ac(:,i) = (f1 - f2)/eps;
    end
    
    % Compute B matrix using finite difference
    f1 = cart_pole_dynamics(0, X, eps, p);
    f2 = cart_pole_dynamics(0, X, 0, p);
    Bc = (f1 - f2)/eps;
    
    % Convert to discrete time Forward Euler approx
    A = eye(n) + dt * Ac;
    B = dt * Bc;
end

%% Cart Pole Plot

function plot_cart_pole(X, p)
    x = X(1);        % Cart position
    theta = X(2);    % Pendulum angle
    l = p.l;         % Pendulum length
    
    % Cart
    cart_w = 0.2;    % Cart width
    cart_h = 0.1;    % Cart height
    cart_x = [x-cart_w/2, x+cart_w/2, x+cart_w/2, x-cart_w/2]; % Cart vertex x
    cart_y = [-cart_h/2, -cart_h/2, cart_h/2, cart_h/2];       % Cart vertex y
    
    % Pendulum endpoints
    pend_x = [x, x + l*sin(theta)];
    pend_y = [0, -l*cos(theta)];
    
    % Plot
    hold on;
    fill(cart_x, cart_y, 'b');                                  % Draw and fill cart
    plot(pend_x, pend_y, 'k', 'LineWidth', 2);                  % Draw pendulum
    plot(pend_x(2), pend_y(2), 'ro', 'MarkerFaceColor', 'r');   % Drawball at top of pendulum
    axis equal;
    grid on;
end
