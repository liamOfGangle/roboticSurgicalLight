% Set up for RigidBodyTree, weights, homeConfig, zeroConfig and IK object 
close all; clearvars;
%% import urdf to make RigidBodyTree
eva = importrobot('C:\Users\ljtov\Documents\roboticSurgicalLight\urdf\newSimpleEVA.urdf');
%% setup zeroConfig struct
zeroConfig = homeConfiguration(eva);
%% weights
weights = ones(1,6); % Pose and orientation equally important
%% ik solver object
ik = inverseKinematics('RigidBodyTree', eva);
%% save workspace as .mat file
% Only run this section if changes have been made to variables above or
% eva.mat does not exist on pc
% save('C:\Users\ljtov\Documents\roboticSurgicalLight\eva.mat');
