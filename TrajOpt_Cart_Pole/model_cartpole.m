function  model = model_cartpole()
    % Creates a cart-pole model for simulation and optimization
    % Returns: model struct containing system dynamics and visualization properties

    persistent last_model; % Caches model to avoid rebuilding
    
    if length(last_model) ~= 0
      model = last_model;
      return
    end
    
    % Physical parameters
    m_cart = 1;        % Mass of cart (kg)
    m_pole = 0.5;      % Mass of pole (kg)
    dim_cart = [0.2 0.1 0.1];  % Cart dimensions [length width height] (m)
    len_pole = 0.7;    % Length of pole (m)
    Iyy_pole = 0.01;   % Moment of inertia of pole around Y axis (kg*m^2)
    
    % Visualization colors (RGB normalized)
    red = [255, 0, 0]/255;
    green = [21, 210, 97]/255;
    white = [255, 255, 255]/255;
    blue = [0, 102, 255]/255;
    yellow = [255, 255, 0]/255;
    
    % Initialize robot model
    model.NB = 2;      % Number of bodies
    model.Nu = 1;      % Number of actuated joints (only cart is actuated)
    model.Bmat = [1;0];  % Actuation matrix (force only applied to cart)

    % Define cart (body 1)
    nb = 1;
    model.parent(nb) = nb - 1;     % No parent (base)
    model.jtype(nb) = {'Px'};      % Prismatic joint along X axis
    model.Xtree(nb) = {eye(6)};    % Spatial transform from parent
    model.I(nb) = { mcI( m_cart, [0 0 0], diag([0 0 0]) ) };  % Inertia properties
    model.appearance.body{nb} = ...
        { 'colour', blue, 'box', [-0.5*dim_cart; 0.5*dim_cart]};
    
    % Define pole (body 2)
    nb = nb + 1;
    model.parent(nb) = nb - 1;     % Connected to cart
    model.jtype(nb) = {'Ry'};      % Rotary joint around Y axis
    model.Xtree(nb) = {plux(eye(3), [0; 0; 0])};  % Spatial transform from cart
    model.I(nb) = { mcI( m_pole, [0 0 -0.5*len_pole], diag([0 Iyy_pole 0]) ) };
    model.appearance.body{nb} = ...
        { 'colour', green, 'cyl', [0 0 0; 0 0 -len_pole], 0.02};
    
    % Setup visualization properties
    model.appearance.base = ...
        { 'tiles', [-1 1; -0.8 0.8; -1 -1], 0.1, ...
        'colour', red, ...
        'cyl', [-1 0 0; 1 0 0], 0.02};

    % Camera settings for visualization
    model.camera.body = 0;
    model.camera.direction = [0 -0.5 0.2];
    model.camera.locus = [0 0];
    model.camera.zoom = 0.5;
    
    % model.gc.point = [0; 0; 0];
    % model.gc.body = 1;

    last_model = model;  % Cache model for future use