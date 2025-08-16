clear all;

addpath(genpath('spatial_v2_extended'))
if ismac
    addpath(genpath('casadi_3.6.7_osx_arm64'))
elseif ispc
    addpath(genpath('casadi-3.6.4-windows64-matlab2018b'))
elseif isunix 
    addpath(genpath('casadi-3.6.4-linux64-matlab2018b'))
else
    disp('Platform not supported')
end