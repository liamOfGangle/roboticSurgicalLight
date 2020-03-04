function [finalJointAngles] = evaIKSoln(eeCoordinates,theta,phi,initialJointAngles)
%evaIKSoln IK solver for Eva given coordinates and angles (in radians)
    %% 
    ee = 'endEffector';
    %% Repopulate zeroConfig with current joint angles
    initialGuess = zeroConfig;
    for i = 1:length(initialGuess)
        initialGuess(i).JointPosition = initialJointAngles(i);
    end
    %% Create transformation matrices
    initialGuess = getTransform(eva, initialGuess, ee); % Initial guess homogeneous transformation matrix
        
    % Rotate z -> y -> x
    % phi is positive anti-clockwise around z-axis
    % theta is positve anti-clockwise around y-axis
    % x-axis rotation points z-axis into centre of sphere
    rotTform = eul2tform([phi theta pi]);
    % Translation
    tranTform = trvec2tform(eeCoordinates);
    % Transformation homogeneous
    tform = tranTform*rotTform;    
    %% Calculate IK 
    [configSoln,solnInfo]=ik(ee,tform,weights,initialGuess);
    %%
    finalJointAngles = zeros(size(configSoln));
    for i = 1:length(configSoln)
        finalJointAngles(i) = configSoln(i).JointPosition;
    end
    %%
%     show(eva, configSoln);
%     hold on;
%     plot3(eeCoordinates(1), eeCoordinates(2), eeCoordinates(3), 'o');
end