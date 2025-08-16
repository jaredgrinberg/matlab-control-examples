% Add parent directory to path for CasADi and Spatial_v2
addpath(genpath('..'))
run ../setup.m
import casadi.*

% Setup optimization problem parameters
p.robot = model_cartpole();  % Get robot model
nq = p.robot.NB;    % Number of configuration variables
nu = 1;             % Number of control inputs
hor = 50;           % Horizon length (timesteps)
dt = 0.04;          % Timestep size (seconds)
Umax = 10;          % Maximum control force (N)

% Create optimization variables
opti = casadi.Opti();  % Initialize optimizer
X = opti.variable(2*nq, hor);    % State trajectory [position; velocity]
U = opti.variable(nu, hor+1);    % Control inputs
obj = MX(0);                     % Initialize objective function

% Define boundary conditions
q0 = [0; 0];        % Initial position [cart; pole]
dq0 = [0; 0];       % Initial velocity
qd = [0; pi];       % Desired position (pole up)
X0 = [q0; dq0];     % Initial state
Xd = [qd; 0;0];     % Desired state

%% Optimization formulation
for kk = 1:hor
   % State error cost: Penalizes deviation from target state
   eX = Xd - X(:,kk);           % State error
   obj = obj + eX' * eX;        % Quadratic state cost

   % Control cost: Penalizes large control inputs (smoothing term)
   obj = obj + 1e-2 * U(:,kk)' * U(:,kk);  % Control regularization

   % Dynamics constraints using trapezoidal integration:
   % This enforces that the state evolution follows physics
   if kk == 1
       % First timestep (t=0 to t=dt)
       % Integrates from initial condition X0 to first state X(:,1)
       fk = dynamics(X0, U(:,1), p);                        % Dynamics at t=0
       fkp1 = dynamics(X(:,1), U(:,2), p);                  % Dynamics at t=dt
       opti.subject_to(X(:,1) == X0 + dt/2 * (fk + fkp1));  % Trapezoidal rule
   elseif kk == hor
       % Final timestep: Enforce target state
       opti.subject_to(X(:,kk) == Xd);                         % Must reach desired state
       fkm1 = dynamics(X(:,kk-1), U(:,kk), p);                 % Dynamics at previous state
       fk = dynamics(X(:,kk), 0*U(:,kk+1), p);                 % Dynamics at final state (zero control)
       opti.subject_to(X(:,kk) == X(:,kk-1) + dt/2 * (fkm1 + fk));  % Integration
   else
       % Middle timesteps: Standard integration
       fkm1 = dynamics(X(:,kk-1), U(:,kk), p);                 % Previous state dynamics
       fk = dynamics(X(:,kk), U(:,kk+1), p);                   % Current state dynamics
       opti.subject_to(X(:,kk) == X(:,kk-1) + dt/2 * (fkm1 + fk));  % Integration step
   end

   % Add control bounds
   opti.subject_to(U(:,kk) <= Umax);
   opti.subject_to(U(:,kk) >= -Umax);
end

% Setup and solve optimization
opti.minimize(obj);
opti.solver('ipopt');
sol = opti.solve();

%% Visualization and plotting
t = (1:hor) * dt;           % Time vector
state = sol.value(X);       % Get optimal state trajectory
qout = state(1:p.robot.NB,:);  % Extract positions
control = sol.value(U);     % Get control inputs
control = control(1:end-1); % Extract controls

% Show animation
showmotion(p.robot, t, qout);

% Plot states and control
close all
figure
subplot(2,1,1)
plot(t,state)
legend('x','th','dx','dth')
subplot(2,1,2)
plot(t,control)
legend('F')

%% Save results
save('cartpole.mat', 't', 'state', 'control','dt','hor')

%% Helper function for dynamics
function dxdt = dynamics(x,u,p)
   % Computes system dynamics using rigid body formulation
   % x: state [position; velocity], u: control input, p: parameters
   robot = p.robot;
   nq = robot.NB;
   
   % Extract states
   q = x(1:nq);       % Positions
   dq = x(nq+1:end);  % Velocities
   
   % Compute accelerations using inverse dynamics
   B = [1; 0];        % Input mapping matrix
   tau = B * u;       % Convert input force to joint torques
   ddq = FDcrb(p.robot, q, dq, tau);  % Forward dynamics - does HandC
   
   dxdt = [dq; ddq];  % Return state derivatives
end