function [robot, homeConfig] = loadURDF(filename, jointAngles)
%loadURDF Loads urdf file for robotic arm
%   Loads .urdf for EVA robotic arm and set home position to a more balanced postion. 

% Load .urdf to make a rigidBodyTree 
robot = importrobot(filename);

% Change home config angles to a more balanced position
homeConfig = homeConfiguration(robot);
for i = 1:length(jointAngles)
    
    if jointAngles(i) ~= 0
        jointAngles(i) = jointAngles(i) + 90;
    end
    homeConfig(i).JointPosition = deg2rad(jointAngles(i)); 
end

