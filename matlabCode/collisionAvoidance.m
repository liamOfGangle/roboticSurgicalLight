close all; clearvars;
load eva.mat;
%%
[theta, phi] = deal(0.0);

cRobot = [0.0, 0.0, 0.187 + 0.096];
rRobot = 0.6 - 0.104;

[bottUp, fCoordsBU, allBottUp] = loadData('handBottomUp2.csv');
[diagFront, fCoordsDF, allDiagFront] = loadData('handDiagonalSide2.csv');
[front, fCoordsF, allFront] = loadData('handFront3.csv');
[obscure, fCoordsO, allObscure] = loadData('handObscure2.csv');
[sideSide, fCoordsSS, allSS] = loadData('handSideSide3.csv');
[topDown, fCoordsTD, allTopDown] = loadData('handTopDown3.csv');

cFocal = mean(vertcat(fCoordsBU, fCoordsDF, fCoordsF, fCoordsO, fCoordsSS, fCoordsTD));
rFocal = 1;

hAngles = [0, 0.5235, -1.7453, 0, -1.9198, 0]; % Pick and place home angles
%%
[rFocal, theta, phi, eeCoords] = calcEndEff(cFocal, rFocal);
[configSoln,currentAngles] = evaIKSolnMATLAB(eeCoords, theta, phi, hAngles);
handRadius = 0.1;
[X,Y,Z] = sphere();
X = X*handRadius; Y = Y*handRadius; Z = Z*handRadius;

CO(:,:,1) = zeros(length(X));
CO(:,:,2) = zeros(length(X));
CO(:,:,3) = ones(length(X));
% %%
% figure();
% show(eva,configSoln);
% hold on;
% object = mesh(X+sideSide(4500,1),Y+sideSide(4500,2),Z+sideSide(4500,3),CO);
% focal = plot3([eeCoords(1) cFocal(1)],[eeCoords(2) cFocal(2)],[eeCoords(3) cFocal(3)],'k','LineWidth',1.5);
% cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
% title('Collision with an object');
% legend([object,cF,focal],{'Hand object','Focal point','Focal vector'},'location','northwest');
% xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
% xlim auto
% ylim auto
% zlim auto
% %%
% figure();
% show(eva,configSoln);
% hold on;
% startData = plot3(sideSide(1,1),sideSide(1,2),sideSide(1,3),'b^','MarkerSize',10);
% endData = plot3(sideSide(end,1),sideSide(end,2),sideSide(end,3),'b*','MarkerSize',10);
% movementData = plot3(sideSide(:,1),sideSide(:,2),sideSide(:,3),'b','LineWidth',1.5);
% focal = plot3([eeCoords(1) cFocal(1)],[eeCoords(2) cFocal(2)],[eeCoords(3) cFocal(3)],'k','LineWidth',1.5);
% cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
% title('Movement data of collision object');
% xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
% legend([startData,endData,movementData,cF,focal],{'Start of movement','End of movement','Movement path','Focal point','Focal vector'},'Location','northwest');
% xlim auto
% ylim auto
% zlim auto
%% Collision avoidance
handData = diagFront;
[rows,cols] = size(handData);

[rFocal,theta,phi,eeCoords] = calcEndEff(cFocal,rFocal);
[configSoln, configAngles] = evaIKSolnMATLAB(eeCoords, theta, phi, hAngles);

minDistBetween = handRadius + rRobot;

figure();

show(eva,configSoln);
hold on;
% handSphere = mesh(X+handData(1,1),Y+handData(1,2),Z+handData(1,3),CO);
focalVector = plot3([cFocal(1) eeCoords(1)],[cFocal(2) eeCoords(2)],[cFocal(3) eeCoords(3)],'k','LineWidth',1.5);


allEE = [];
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
        
        allEE = vertcat(allEE,eeCoords);
        
%         plot3(eeCoords(1),eeCoords(2),eeCoords(3),'o');                     
    end
end

% [rFocal, theta, phi, eeCoords] = calcEndEff(cFocal, rFocal);
% [configSoln, ~] = evaIKSolnMATLAB(eeCoords, theta, phi, hAngles);

[rows,~] = size(allEE);
for j = 1:rows
    
%     if mod(j,6) == 0 || j == 1
    
        [theta,phi] = calcThetaPhi(allEE(j,:),cFocal,rFocal);
        [configSoln, configAngles] = evaIKSolnMATLAB(allEE(j,:), theta, phi, configAngles);
        show(eva,configSoln);
        focalVector = plot3([cFocal(1) allEE(j,1)],[cFocal(2) allEE(j,2)],[cFocal(3) allEE(j,3)],'k','LineWidth',1.5);
%     end
end 

dataHand = plot3(handData(:,1),handData(:,2),handData(:,3),'b','LineWidth',1.5);
startData = plot3(handData(1,1),handData(1,2),handData(1,3),'b^','MarkerSize',10);
endData = plot3(handData(end,1),handData(end,2),handData(end,3),'b*','MarkerSize',10);

eeData = plot3(allEE(:,1),allEE(:,2),allEE(:,3),'Color','#EDB120','LineWidth',1.5);
eeStart = plot3(allEE(1,1),allEE(1,2),allEE(1,3),'^','Color','#EDB120','MarkerSize',10);
eeEnd = plot3(allEE(end,1),allEE(end,2),allEE(end,3),'*','Color','#EDB120','MarkerSize',10);

cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);

title('Avoiding collison');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
legend([startData,endData,dataHand,eeStart,eeEnd,eeData,cF,focalVector],{'Start of movement','End of movement','Movement path','Start of robot movement','End of robot movement','Robot movement','Focal point','Focal vector'},'Location','northwest');
xlim auto
ylim auto
zlim auto