% Purpose: Computes forward kinematics and dynamics for an N-link robotic manipulator
% Outputs: Generates two MATLAB functions for:
%   1. Forward kinematics
%   2. System dynamics (mass matrix, Coriolis, gravity)

% Clear MATLAB workspace and figures
clear all;
close all;

%% System Configuration
% Define number of links in the robotic chain
N = 2;  % Number of links

% Initialize symbolic variables for robot parameters
% Each variable is an Nx1 vector where N is number of links
q = sym('q', [N,1], 'real');    % Joint angles (radians)
dq = sym('dq', [N,1], 'real');  % Joint angular velocities (radians/s)
l = sym('l', [N,1], 'real');    % Link lengths (meters)
r = sym('r', [N,1], 'real');    % Center of mass positions relative to link length (ratio 0 to 1)
M = sym('M', [N,1], 'real');    % Link masses (kg)
J = sym('J', [N,1], 'real');    % Link rotational inertias (kg*m^2)

% Combine all physical parameters into one vector for function generation
params = [l; r; M; J];

% Define 2D rotation matrix function
% Input: angle in radians
% Output: 2x2 rotation matrix
rot2d = @(x) [cos(x) -sin(x); sin(x) cos(x)];

%% Forward Kinematics
% Initialize symbolic arrays for storing kinematic quantities
T = sym('T', [3,3,N]);        % Homogeneous transformation matrices (3x3xN)
pjoint = sym('pjoint', [2,N]); % Joint positions in global frame (2xN)
pcom = sym('pcom', [2,N]);     % Center of mass positions in global frame (2xN)
vcom = sym('vcom', [2,N]);     % Center of mass velocities in global frame (2xN)

% Compute forward kinematics for each link
for ii = 1:N
    % Compute link transformation matrix
    ri = [l(ii); 0];           % Link vector in local frame
    Ri = rot2d(q(ii));         % Link rotation matrix
    Ti = [Ri, Ri*ri;           % Homogeneous transformation matrix for link i
          0, 0, 1];            % [R, R*p; 0, 1] format
    
    % Compute center of mass transformation
    rcomi = [r(ii)*l(ii); 0];  % COM position vector in local frame
    Tcomi = [Ri, Ri*rcomi;     % Homogeneous transformation matrix for COM
             0, 0, 1];
    
    if ii == 1
        % For first link, use direct transformation
        T(:,:,ii) = Ti;                    % Store transformation
        pjoint(:,ii) = Ti(1:2,3);          % Extract joint position
        pcom(:,ii) = Tcomi(1:2,3);         % Extract COM position
    else
        % For subsequent links, multiply with previous transformations
        T_temp = simplify(T(:,:,ii-1) * Ti);     % Cumulative transformation
        T(:,:,ii) = T_temp;                       % Store transformation
        pjoint(:,ii) = T_temp(1:2,3);            % Global joint position
        
        T_temp = simplify(T(:,:,ii-1) * Tcomi);  % Transform COM to global frame
        pcom(:,ii) = T_temp(1:2,3);              % Extract global COM position
    end
    
    % Compute COM velocities using the Jacobian
    vcom(:,ii) = jacobian(pcom(:,ii), q) * dq;
end

%% End-effector Kinematics
% Compute end-effector quantities (last link)
T_ee = T(:,:,N);                  % End-effector transformation matrix
p_ee = T_ee(1:2,3);              % End-effector position
J_ee = jacobian(p_ee, q);         % End-effector Jacobian
v_ee = J_ee * dq;                 % End-effector velocity

% Compute time derivative of Jacobian (needed for acceleration calculations)
dJ_ee = simplify(reshape(jacobian(J_ee(:), q)*dq, size(J_ee)));
dJdq_ee = dJ_ee * dq;            % J_dot * q_dot term

%% Generate Forward Kinematics Function
% Create filename based on number of links
filename = ['fcn_FK_', num2str(N), '_link.m'];

% Collect joint positions into a chain array [base, joint1, joint2, ...]
chain = [[0;0], reshape(T(1:2,3,:), [2,N])];

% Generate MATLAB function for forward kinematics
matlabFunction(p_ee, v_ee, J_ee, dJdq_ee, chain, ...
    'File', filename, ...
    'Vars', {q, dq, params});

%% System Dynamics
% Compute total Center of Mass
CoM = pcom * M / sum(M);    % Weighted average of link COMs
vcom = reshape(jacobian(pcom(:),q) * dq, [2,N]);  % COM velocities

% Compute system Kinetic Energy
KE = sym(0);
for ii = 1:N
    wi = sum(dq(1:ii));  % Cumulative angular velocity up to link i
    
    % KE = 1/2 * m * v^2 (translational) + 1/2 * I * ω^2 (rotational)
    KE = KE + 0.5 * vcom(:,ii)' * M(ii) * vcom(:,ii) + ...
         0.5 * wi' * J(ii) * wi;
end
KE = simplify(KE);

% Compute Potential Energy (PE = mgh)
PE = pcom(2,:) * M * 9.81;  % Using y-coordinate (2nd row) for height

% Compute Mass Matrix (H) using KE = 1/2 * dq' * H * dq
H = simplify(jacobian(jacobian(KE, dq).', dq));

% Compute Gravity Vector (G) from PE
G = jacobian(PE, q)';

% Compute Coriolis Matrix (C) using Christoffel symbols
% C(q,dq) contains centripetal and Coriolis terms
C = sym('C', [N,N], 'real');
for i = 1:N
    for j = 1:N
        C(i,j) = 0;
        for k = 1:N
            % Christoffel symbols of the first kind
            temp = jacobian(H(i,j),q(k)) + ...
                   jacobian(H(i,k),q(j)) - ...
                   jacobian(H(k,j),q(i));
            C(i,j) = C(i,j) + 0.5 * temp * dq(k);
        end
    end
end
C = simplify(C);

% Generate dynamics function
% The equation of motion is: H(q)ddq + C(q,dq)dq + G(q) = tau
filename = ['fcn_dynamics_', num2str(N), '_link.m'];
matlabFunction(H, C, G, 'File', filename, 'Vars', {q, dq, params});