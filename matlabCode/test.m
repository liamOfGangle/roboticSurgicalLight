clear all, close all
load eva.mat
show(eva,zeroConfig);

phi = deg2rad(45); % rotInXY
theta = deg2rad(45); % rotAroundZ
r = 0.25; 

xFocal = -0.3; yFocal = 0.3; zFocal = 0.0;

% If theta > 0, rotate Z -> Y -> X, all positive. 
% If theta < 0, rotate Z -> X -> Y, X rot must be negative.

firstRot = axang2rotm([0 0 1 phi]);
secondRot = axang2rotm([0 1 0 theta]);
thirdRot = axang2rotm([1 0 0 pi]);

rotTfrom = rotm2tform(firstRot*secondRot*thirdRot);

xEndPos = xFocal + r*sin(theta)*cos(phi);
yEndPos = yFocal + r*sin(theta)*cos(phi);
zEndPos = zFocal + r*cos(theta);

transTform = trvec2tform([xEndPos yEndPos zEndPos]);

tform = transTform*rotTfrom;

[configSoln,solnInfo] = ik('endEffector',tform,weights,zeroConfig);
 
hold on
show(eva,configSoln);
plot3(xFocal, yFocal, zFocal, 'o');

