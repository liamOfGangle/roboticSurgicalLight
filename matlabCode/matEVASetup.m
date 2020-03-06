% Set up for RigidBodyTree, weights, homeConfig, zeroConfig and IK object 
close all; clearvars;
%% import urdf to make RigidBodyTree
eva = importrobot('C:\Users\ljtov\Documents\roboticSurgicalLight\urdf\newSimpleEVA.urdf');
%% setup zeroConfig struct and homeConfig struct
zeroConfig = homeConfiguration(eva);
%% weights
weights = [1 1 1 0.8 0.8 0.8];
%% ik solver object
ik = inverseKinematics('RigidBodyTree', eva);
%% save workspace as .mat file
save('C:\Users\ljtov\Documents\roboticSurgicalLight\eva.mat');
