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
cFocal1 = cFocal + [-0.25 0 0.25];
rFocal = 1;

[X,Y,Z] = sphere();
rCO(:,:,1) = zeros(length(X));
rCO(:,:,2) = ones(length(X))*0.447;
rCO(:,:,3) = ones(length(X))*0.741;

fCO(:,:,1) = ones(length(X))*0.85;
fCO(:,:,2) = ones(length(X))*0.325;
fCO(:,:,3) = ones(length(X))*0.098;

hAngles = [0, 0.5235, -1.7453, 0, -1.9198, 0]; % Pick and place home angles
homeConfig = zeroConfig;
for j = 1:length(hAngles)
    homeConfig(j).JointPosition = hAngles(j);
end
%%

%% Sphere plots
% Spheres not intersecting 
figure();

rX = (rRobot*X)+cRobot(1);
rY = (rRobot*Y)+cRobot(2);
rZ = (rRobot*Z)+cRobot(3);

fX = (rFocal*X)+cFocal1(1) - 1.25;
fY = (rFocal*Y)+cFocal1(2);
fZ = (rFocal*Z)+cFocal1(3);

rSpace = mesh(rX,rY,rZ,rCO);
hold on;
fSpace = mesh(fX,fY,fZ,fCO);
hold off;

title({'Sphere-Sphere interactions:';'No intersection'});
legend([rSpace,fSpace],{'Dextrous workspace of arm','Focal sphere of radius 1 meter'},'location','northwest');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
xlim auto
ylim auto
zlim auto

% Sphere intersection
figure();
rX = (rRobot*X)+cRobot(1);
rY = (rRobot*Y)+cRobot(2);
rZ = (rRobot*Z)+cRobot(3);

fX = (rFocal*X)+cFocal1(1);
fY = (rFocal*Y)+cFocal1(2);
fZ = (rFocal*Z)+cFocal1(3);

rSpace = mesh(rX,rY,rZ,rCO);
hold on;
fSpace = mesh(fX,fY,fZ,fCO);
hold off;

title({'Sphere-Sphere interactions:';'Intersection'});
legend([rSpace,fSpace],{'Dextrous workspace of arm','Focal sphere of radius 1 meter'},'location','northwest');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
xlim auto
ylim auto
zlim auto

% Intersection circle
figure();
rho = deg2rad(linspace(0,360,361));
d = norm(cFocal1 - cRobot);
alpha = 0.5 + ((rRobot^2 - rFocal^2)/(2*(d^2)));
ci = cRobot + alpha*(cFocal1 - cRobot); % Centre of intersect circle
ri = sqrt(rRobot^2 - (alpha*d)^2); % Radius of intersect circle

normCC = (cFocal1 - cRobot)/d; % Normalised vector that runs perpendicular to intersect circle
        
xu = (-1.0*(normCC(3) + normCC(2)))/normCC(1); % x value of tangent
yu = (-1.0*(normCC(3) + normCC(1)))/normCC(2); % y value of tangent
zu = (-1.0*(normCC(2) + normCC(1)))/normCC(3); % z value of tangent

U = [xu, 1, 1]; % Tangent vector U
normU = U/norm(U); % Normalised U vector

if any(normU > 1.0)
    U = [1, yu, 1];
    normU = U/norm(U);

    if any(normU > 1.0)
        U = [1, 1, zu];
        normU = U/norm(U);

        if any(normU > 1.0)
            error('Element/s of normU are >1')
        end
    end
end

V = cross(normCC, normU); % bitangent vector

zMax = -inf;
interP = [];
for j = 1:length(rho)
    P = ci + ri*(normU*cos(rho(j)) + V*sin(rho(j)));
    interP = vertcat(interP,P);
    if P(3) > zMax
        zMax = P(3);
        bestCaseCoords = P;
    end
end

interCircle = plot3(interP(:,1),interP(:,2),interP(:,3),'m','LineWidth',3);
hold on;
cR = plot3(cRobot(1),cRobot(2),cRobot(3),'x','Color',[0 0.447 0.741],'MarkerSize',10);
cF = plot3(cFocal1(1),cFocal1(2),cFocal1(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
bestCoords = plot3(bestCaseCoords(1),bestCaseCoords(2),bestCaseCoords(3),'ko','MarkerFaceColor','k');
focalVector = plot3([bestCaseCoords(1) cFocal1(1)],[bestCaseCoords(2) cFocal1(2)],[bestCaseCoords(3) cFocal1(3)],'k','LineWidth',1.5);
hold off;
grid on;

title('Intersection circle');
legend([interCircle,cR,cF,bestCoords,focalVector],{'Intersection circle of two spheres','Centre of robot workspace','Focal point','Coordinates with min theta from [0 0 1]','Focal vector'},'location','northwest');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
xlim auto
ylim auto
zlim auto

% With robot
figure();
[theta,phi] = calcThetaPhi(bestCaseCoords,cFocal1,rFocal);
[configSoln,finalJointAngles] = evaIKSolnMATLAB(bestCaseCoords,theta,phi,hAngles);
show(eva,configSoln);
hold on;

interCircle = plot3(interP(:,1),interP(:,2),interP(:,3),'m','LineWidth',3);

cR = plot3(cRobot(1),cRobot(2),cRobot(3),'x','Color',[0 0.447 0.741],'MarkerSize',10);
cF = plot3(cFocal1(1),cFocal1(2),cFocal1(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
% bestCoords = plot3(bestCaseCoords(1),bestCaseCoords(2),bestCaseCoords(3),'ko','MarkerFaceColor','k');
focalVector = plot3([bestCaseCoords(1) cFocal1(1)],[bestCaseCoords(2) cFocal1(2)],[bestCaseCoords(3) cFocal1(3)],'k','LineWidth',1.5);
hold off;
grid on;

title({'Intersection circle';'with robot model'});
legend([interCircle,cR,cF,focalVector],{'Intersection circle of two spheres','Centre of robot workspace','Focal point','Focal vector'},'location','northwest');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
xlim auto
ylim auto
zlim auto

%% URDF plot
figure();
show(eva,homeConfig);
title('Model of EVA robotic arm in home configuration');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
xlim auto
ylim auto
zlim auto
%%
figure();
[rFocal, theta, phi, eeCoords] = calcEndEff(cFocal, rFocal);
[configSoln,fJA] = evaIKSolnMATLAB(eeCoords, theta, phi, hAngles);
show(eva,configSoln);
hold on;
p1 = plot3(bottUp(:,1), bottUp(:,2), bottUp(:,3),'LineWidth',1.5);
p2 = plot3(diagFront(:,1), diagFront(:,2), diagFront(:,3),'LineWidth',1.5);
p3 = plot3(front(:,1), front(:,2), front(:,3),'LineWidth',1.5);
p4 = plot3(obscure(:,1), obscure(:,2), obscure(:,3),'LineWidth',1.5);
p5 = plot3(sideSide(:,1), sideSide(:,2), sideSide(:,3),'LineWidth',1.5);
p6 = plot3(topDown(:,1), topDown(:,2), topDown(:,3),'LineWidth',1.5);
hold off;
legend([p1,p2,p3,p4,p5,p6],{'1','2','3','4','5','6'})
%%
figure();
show(eva,configSoln);
hold on;
p4 = plot3(obscure(:,1), obscure(:,2), obscure(:,3),'LineWidth',1.5);
cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
hold off;
legend([p4,cF],{'Hand movement','Focal point'},'location','northwest');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
title('Data of a hand obstructing the focal point');
xlim auto
ylim auto
zlim auto
%% 
figure();
[rFocal, theta, phi, eeCoords] = calcEndEff(cFocal, rFocal);
[configSoln,fJA] = evaIKSolnMATLAB(eeCoords, theta, phi, hAngles);
handRadius = 0.1;
[X,Y,Z] = sphere();
X = X*handRadius; Y = Y*handRadius; Z = Z*handRadius;

CO(:,:,1) = zeros(length(X));
CO(:,:,2) = zeros(length(X));
CO(:,:,3) = ones(length(X));

show(eva,configSoln);
hold on; 
focalVector = plot3([eeCoords(1) cFocal(1)],[eeCoords(2) cFocal(2)],[eeCoords(3) cFocal(3)],'k','LineWidth',1.5);
object = mesh(X+obscure(40000,1),Y+obscure(40000,2),Z+obscure(40000,3),CO);
cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
hold off;
legend([object,focalVector,cF],{'Hand object','Focal vector','Focal point'},'location','northwest');
xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
title('Obstructing the focal point');
xlim auto
ylim auto
zlim auto
%%
figure();
d = norm(cFocal - cRobot); % Distance between centre of both spheres
eeCoords = cFocal + [0.0, 0.0, rFocal]; % Assume directly above

% First case 
if norm(eeCoords - cRobot) < rRobot
    rho = deg2rad(linspace(0,360,360+1));
    points = [];
    for j = 1:length(rho)
        pRho = cFocal + rFocal*([0,1,0]*cos(j)+[0,0,1]*sin(j));
        points = vertcat(points,pRho);
    end
end
% show(eva,configSoln);
% hold on;
% plot3(points(:,1), points(:,2), points(:,3),'x');
    [r,~] = size(points);
    minTheta = inf;
    c = obscure(40000,:);
    for j = 1:r
        o = points(j,:);
        dHat = (cFocal - o)/norm(cFocal - o);
        discriminant = (dot(dHat,(o - c)))^2 - ((norm(o - c))^2 - rFocal^2);
        if discriminant < 0
            [theta, phi] = calcThetaPhi(o, cFocal, rFocal);
            disp(rad2deg(theta));
            if theta < minTheta
                minTheta = theta;
                idx = j;
            end
        end
    end

% % Second case
% elseif d < (rRobot + rFocal) && norm(eeCoords - cRobot) > rRobot
%     alpha = 0.5 + ((rRobot^2 - rFocal^2)/(2*(d^2)));
% 
%     ci = cRobot + alpha*(cFocal - cRobot); % Centre of intersect circle
% 
%     ri = sqrt(rRobot^2 - (alpha*d)^2); % Radius of intersect circle
% 
%     [~, normU, V] = calcTangentBitangent(cRobot, cFocal);
% 
%     [points, eeCoords, ~] = calcTanBitanPointsAndVector(ci, normU, V, ri, 'zMax');
%     
%     [r,~] = size(points);
%     minTheta = inf;
%     c = obscure(40000,:);
%     for j = 1:r
%         o = points(r,:);
%         dHat = (cFocal - o)/norm(cFocal - o);
%         discriminant = (dot(dHat,(o - c)))^2 - ((norm(o - c))^2 - rFocal^2);
%         if discriminant >= 0
%             [theta, phi] = calcThetaPhi(o, cFocal, rFocal);
%             if theta < minTheta
%                 minTheta = theta;
%                 idx = j;
%             end
%         end
%     end 
% end
% 
eeCoords = points(idx,:);
[theta, phi] = calcThetaPhi(idx,cFocal,rFocal);

[configSoln,~] = evaIKSolnMATLAB(eeCoords,theta,phi,hAngles);

show(eva,configSoln);



