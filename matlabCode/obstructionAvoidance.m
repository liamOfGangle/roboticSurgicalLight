close all; clearvars;
load eva.mat;
%% Variables 
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
% cFocal1 = cFocal + [-0.25 0 0.25];
rFocal = 1;

hAngles = [0, 0.5235, -1.7453, 0, -1.9198, 0]; % Pick and place home angles
%% 
[rFocal, theta, phi, eeCoords] = calcEndEff(cFocal, rFocal);
[configSoln,fJA] = evaIKSolnMATLAB(eeCoords, theta, phi, hAngles);
handRadius = 0.1;
[X,Y,Z] = sphere();
X = X*handRadius; Y = Y*handRadius; Z = Z*handRadius;

CO(:,:,1) = zeros(length(X));
CO(:,:,2) = zeros(length(X));
CO(:,:,3) = ones(length(X));
%% 
if eeCoords == (cFocal + [0.0, 0.0, rFocal])
    [circle, ~, ~] = calcTanBitanPointsAndVector(cFocal, [0,1,0], [0,0,1], rFocal, 'zMax');
    [r,c] = size(circle);
    circleInWSpace = [];
    
    for j = 1:r
        pointOnCircle = circle(j,:);
        if norm(pointOnCircle - cRobot) <= rRobot
            circleInWSpace = vertcat(circleInWSpace,pointOnCircle);
        end
    end
end

d = norm(cFocal - cRobot);
alpha = 0.5 + ((rRobot^2 - rFocal^2)/(2*(d^2)));
ci = cRobot + alpha*(cFocal - cRobot); % Centre of intersect circle
ri = sqrt(rRobot^2 - (alpha*d)^2); % Radius of intersect circle

[~,tangent,biTangent] = calcTangentBitangent(cRobot,cFocal);
[interCircle,~,~] = calcTanBitanPointsAndVector(ci,tangent,biTangent,ri,'zMax');

%%
figure();
show(eva,configSoln);
hold on; 
focalVector = plot3([eeCoords(1) cFocal(1)],[eeCoords(2) cFocal(2)],[eeCoords(3) cFocal(3)],'k','LineWidth',1.5);
object = mesh(X+obscure(40000,1),Y+obscure(40000,2),Z+obscure(40000,3),CO);
cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
arch = plot3(circleInWSpace(:,1),circleInWSpace(:,2),circleInWSpace(:,3),'LineWidth',1.5,'Color','#EDB120');
circle = plot3(interCircle(:,1),interCircle(:,2),interCircle(:,3),'LineWidth',1.5,'Color','#EDB120');
hold off;
legend([object,focalVector,cF,arch],{'Hand object','Focal vector','Focal point','Possible end effector positions'},'location','northwest');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
title('Obstructing the focal point');
xlim auto
ylim auto
zlim auto
%%
possiblePositions = vertcat(circleInWSpace,interCircle);

[rows,~] = size(possiblePositions);
c = obscure(40000,:);
r = handRadius;
minTheta = inf;

for j = 1:rows
    o = possiblePositions(j,:);
    dHat = (cFocal - o)/norm(cFocal - o);
    
    discrim = (dot(dHat,(o - c))^2 - ((norm(o - c))^2 - r^2));
    
    if discrim < 0
        [theta,~] = calcThetaPhi(o,cFocal,rFocal);
        if theta < minTheta
            minTheta = theta;
            idx = j;
        end
    end
end

bestPos = possiblePositions(idx,:);
[theta,phi] = calcThetaPhi(bestPos,cFocal,rFocal);

[configSoln,~] = evaIKSolnMATLAB(bestPos,theta,phi,hAngles);

%%
figure();

show(eva,configSoln);
hold on; 
focalVector = plot3([bestPos(1) cFocal(1)],[bestPos(2) cFocal(2)],[bestPos(3) cFocal(3)],'k','LineWidth',1.5);
object = mesh(X+obscure(40000,1),Y+obscure(40000,2),Z+obscure(40000,3),CO);
cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
arch = plot3(circleInWSpace(:,1),circleInWSpace(:,2),circleInWSpace(:,3),'LineWidth',1.5,'Color','#EDB120');
circle = plot3(interCircle(:,1),interCircle(:,2),interCircle(:,3),'LineWidth',1.5,'Color','#EDB120');
hold off;
legend([object,focalVector,cF,arch],{'Hand object','Focal vector','Focal point','Possible end effector positions'},'location','northwest');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
title('No longer obstructing the focal point');
xlim auto
ylim auto
zlim auto
        
    


