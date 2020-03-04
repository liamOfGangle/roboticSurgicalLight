% Set up for RigidBodyTree, weights, homeConfig, zeroConfig and IK object 
close all; clearvars;
%% import urdf to make RigidBodyTree
eva = importrobot('C:\Users\ljtov\Documents\roboticSurgicalLight\urdf\newSimpleEVA.urdf');
%% setup zeroConfig struct and homeConfig struct
zeroConfig = homeConfiguration(eva);
homeConfig = homeConfiguration(eva);

% angles in degrees for home angles. Documentation max or min +/- 3 degrees
homeAngles = [0 67 -157 0 -90 0];
% could do in for loop but will parse to python so want to make sure only
% one answer per joint
homeConfig(1).JointPosition = deg2rad(homeAngles(1));
homeConfig(2).JointPosition = deg2rad(homeAngles(2));
homeConfig(3).JointPosition = deg2rad(homeAngles(3));
homeConfig(4).JointPosition = deg2rad(homeAngles(4));
homeConfig(5).JointPosition = deg2rad(homeAngles(5));
homeConfig(6).JointPosition = deg2rad(homeAngles(6));

clear homeAngles
%% weights
weights = [1 1 1 0.8 0.8 0.8];
%% ik solver object
ik = inverseKinematics('RigidBodyTree', eva);
%% save workspace as .mat file
save('C:\Users\ljtov\Documents\roboticSurgicalLight\eva.mat');
