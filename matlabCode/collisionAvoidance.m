close all; clearvars;
%% Load data
load eva.mat;

movementType = 'topToBottom';

switch movementType
    case 'bottomToTop'
        [handData, focalPointCoords, allTrackingData] = loadData('handBottomUp2.csv');
    case 'topToBottom'
        [handData, focalPointCoords, allTrackingData] = loadData('handTopDown3.csv');
    case 'sideToSide'
        [handData, focalPointCoords, allTrackingData] = loadData('handSideSide3.csv');
    case 'directFromFront'
        [handData, focalPointCoords, allTrackingData] = loadData('handFront3.csv');
    case 'diagonalFromFront'
        [handData, focalPointCoords, allTrackingData] = loadData('handDiagonalSide2.csv');
    case 'obsureUnderneath'
        [handData, focalPointCoords, allTrackingData] = loadData('handObscure2.csv');
    otherwise
        error('Not recognised movement cases');
end
%% Variables
cRobot = [0.0, 0.0, 0.187 + 0.096];
rRobot = 0.6 - 0.104;

rFocal = 1;

homeAngles = [0, 0.5235, -1.7453, 0, -1.9198, 0]; % Pick and place home angles
homeConfig = zeroConfig;
for j = 1:length(homeAngles)
    homeConfig(j).JointPosition = homeAngles(j);
end

handRadius = 0.1; 
eeRadius = 0.125; 

[X,Y,Z] = sphere();
X = X*handRadius; Y = Y*handRadius; Z = Z*handRadius;

CO(:,:,1) = zeros(length(X));
CO(:,:,2) = zeros(length(X));
CO(:,:,3) = ones(length(X));
%% Figure to show hand movement data
figure();

% focalPointCoords = focalPointCoords + [-1 0 0];

[rFocal, theta, phi, eeCoords] = calcEndEff(focalPointCoords, rFocal);
[configSoln, ~] = evaIKSolnMATLAB(eeCoords, theta, phi, homeAngles);

show(eva, configSoln); hold on;

startData = plot3(handData(1,1),handData(1,2),handData(1,3),'b^','MarkerSize',10);
endData = plot3(handData(end,1),handData(end,2),handData(end,3),'b*','MarkerSize',10);
movementData = plot3(handData(:,1),handData(:,2),handData(:,3),'b','LineWidth',1.5);
focalPosition = plot3(focalPointCoords(1),focalPointCoords(2),focalPointCoords(3),'kx','MarkerSize',10);
focalVector = plot3([focalPointCoords(1) eeCoords(1)],[focalPointCoords(2) eeCoords(2)],[focalPointCoords(3) eeCoords(3)],'k','LineWidth',1.5);

title('Movement data');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
legend([startData,endData,movementData,focalPosition,focalVector],{'Start of movement','End of movement','Movement path','Focal point','Focal vector'},'Location','northwest');
%% Collision avoidance
[rows,cols] = size(handData);

[rFocal,theta,phi,eeCoords] = calcEndEff(focalPointCoords,rFocal);
[configSoln, configAngles] = evaIKSolnMATLAB(eeCoords, theta, phi, homeAngles);

minDistBetween = handRadius + eeRadius;

figure();

show(eva,configSoln);
hold on;
handSphere = mesh(X+handData(1,1),Y+handData(1,2),Z+handData(1,3),CO);
focalVector = plot3([focalPointCoords(1) eeCoords(1)],[focalPointCoords(2) eeCoords(2)],[focalPointCoords(3) eeCoords(3)],'k','LineWidth',1.5);

for j = 2:rows
    k = j - 1;
    currPos = handData(j,:);
    prevPos = handData(k,:);

    [dirVec,predPos] = predictedPointAndDirectionVector(prevPos,currPos);

    eeObjVec = eeCoords - predPos;
    eeObjDist = norm(eeObjVec);

    axisVec = [0 0 0];

    if eeObjDist < minDistBetween

        closestAxis = dominantAxis(prevPos,currPos,30);

        if strcmp("y axis",closestAxis) || strcmp("z axis",closestAxis)
            scalingFactor = norm(dirVec);
            axisVec = scalingFactor*[1 0 0];
        elseif strcmp("x axis",closestAxis)
            scalingFactor = norm(dirVec);
            axisVec = scalingFactor*[0 1 0];
        end
        
        overLapDist = minDistBetween - eeObjDist;
        overLapVec = (overLapDist/eeObjDist)*eeObjVec;
        
        eeCoords = eeCoords + (axisVec + dirVec + overLapVec);
        
        inWorkspaceVec = cRobot - eeCoords;
        inWorkspaceDist = norm(inWorkspaceVec);
        if inWorkspaceDist > rRobot
            distToWorkspace = inWorkspaceDist - rRobot;
            eeCoords = eeCoords + (distToWorkspace/inWorkspaceDist)*inWorkspaceVec;
        end
        
        plot3(eeCoords(1),eeCoords(2),eeCoords(3),'o');                     
    end
end

[rFocal, theta, phi, eeCoords] = calcEndEff(focalPointCoords, rFocal);
[configSoln, ~] = evaIKSolnMATLAB(eeCoords, theta, phi, homeAngles);