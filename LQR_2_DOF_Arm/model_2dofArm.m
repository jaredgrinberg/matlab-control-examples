function model = model_2dofArm()
    % Parameters
    m_link1 = 0.5;
    m_link2 = 1;
    len_link1 = 0.5;
    len_link2 = 1;
    Iyy_link1 = 1e-3;
    Iyy_link2 = 2e-3;
    
    % Colors
    red = [255, 0, 0]/255;
    green = [21, 210, 97]/255;
    white = [255, 255, 255]/255;
    blue = [0, 102, 255]/255;
    yellow = [255, 255, 0]/255;
    
    % Build robot
    model.NB = 2;
    nb = 1;
    gc_body = [];
    gc_point = [];
    
    model.gravity = [0; -9.81];
    
    % 1 - link1
    model.parent(nb) = nb - 1;
    model.jtype(nb) = {'r'};
    model.Xtree(nb) = {eye(3)};
    model.I(nb) = {mcI(m_link1, [0.5*len_link1 0], Iyy_link1)};
    model.appearance.body{nb} = ...
        {'colour', blue, 'cyl', [0 0 0; len_link1 0 0], 0.02};
    
    % 2 - link2
    nb = nb + 1;
    model.parent(nb) = nb - 1;
    model.jtype(nb) = {'r'};
    model.Xtree(nb) = {plnr(0, [len_link1 0])};
    model.I(nb) = { mcI(m_link2, [0.5*len_link2 0], Iyy_link2)};
    model.appearance.body{nb} = ...
        { 'colour', red, 'cyl', [0 0 0; len_link2 0 0], 0.02};
    
    % Drawing instructions
    model.appearance.base = ...
        { 'tiles', [-5 5; -2 2; -0.1 -0.1], 0.5};
    
    % Camera settings
    model.camera.body = 0;
    model.camera.direction = [0 -0.5 5];
    model.camera.locus = [0 0];
    model.camera.zoom = 0.4;
end