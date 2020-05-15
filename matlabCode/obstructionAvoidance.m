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
[configSoln,currentAngles] = evaIKSolnMATLAB(eeCoords, theta, phi, hAngles);
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

[ci,ri] = calcIntersectCentreRadius(cFocal,cRobot,rFocal,rRobot);
[~,tangent,biTangent] = calcTangentBitangent(cRobot,cFocal);
[interCircle,~,~] = calcTanBitanPointsAndVector(ci,tangent,biTangent,ri,'zMax');

possiblePoints = vertcat(circleInWSpace,interCircle);

%%
% obscure = sideSide + [0, 0, -0.5];
figure();
show(eva,configSoln);
hold on;
plot3(obscure(:,1), obscure(:,2), obscure(:,3),'LineWidth',1.5);
[rows,~] = size(obscure);
[r,~] = size(possiblePoints);
allEE = [];
for j = 1:rows
    discriminant = lineSphereDiscriminant(eeCoords,cFocal,obscure(j,:),handRadius);
    if discriminant >= 0
        minTheta = inf;
        
        for k = 1:r
            discriminant = lineSphereDiscriminant(possiblePoints(k,:),cFocal,obscure(j,:),handRadius);
            if discriminant < 0
                [theta,~] = calcThetaPhi(possiblePoints(k,:),cFocal,rFocal);

                if theta < minTheta
                    minTheta = theta;
                    idx = k;
                end
            end
        end
        
        eeCoords = possiblePoints(idx,:);
        [theta,phi] = calcThetaPhi(eeCoords,cFocal,rFocal);
        [configSoln,currentAngles] = evaIKSolnMATLAB(eeCoords,theta,phi,currentAngles);
        allEE = vertcat(allEE,eeCoords);
        show(eva,configSoln);
        xlim auto
        ylim auto
        zlim auto
    end
end
% [r,c] = size(allEE);
% disp(r);
%% Results plots
% [rFocal, theta, phi, eeCoords] = calcEndEff(cFocal, rFocal);
% [configSoln,currentAngles] = evaIKSolnMATLAB(eeCoords, theta, phi, hAngles);
% 
% for j = 1:rows
%     discriminant = lineSphereDiscriminant(eeCoords,cFocal,obscure(j,:),handRadius);
%     if discriminant >= 0
%         minTheta = inf;
%         
%         for k = 1:r
%             discriminant = lineSphereDiscriminant(possiblePoints(k,:),cFocal,obscure(j,:),handRadius);
%             if discriminant < 0
%                 [theta,~] = calcThetaPhi(possiblePoints(k,:),cFocal,rFocal);
% 
%                 if theta < minTheta
%                     minTheta = theta;
%                     idx = k;
%                 end
%             end
%         end
%         
%         figure();
%         show(eva,configSoln);
%         hold on;
%         focalVector = plot3([eeCoords(1) cFocal(1)],[eeCoords(2) cFocal(2)],[eeCoords(3) cFocal(3)],'k','LineWidth',1.5);
%         object = mesh(X+obscure(j,1),Y+obscure(j,2),Z+obscure(j,3),CO);
%         cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
%         handData = plot3(obscure(:,1), obscure(:,2), obscure(:,3),'b','LineWidth',1.5);
%         startData = plot3(obscure(1,1),obscure(1,2),obscure(1,3),'b^','MarkerSize',10);
%         endData = plot3(obscure(end,1),obscure(end,2),obscure(end,3),'b*','MarkerSize',10);
%         legend([startData,endData,handData,object,cF,focalVector],{'Start of object movement','End of object movement','Object movement path','object','Focal point','Focal vector'},'Location','northwest');
%         xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
%         xlim auto
%         ylim auto
%         zlim auto
%         view(-135,25);
%         
%         eeCoords = possiblePoints(idx,:);
%         [theta,phi] = calcThetaPhi(eeCoords,cFocal,rFocal);
%         [configSoln,currentAngles] = evaIKSolnMATLAB(eeCoords,theta,phi,currentAngles);
%         
%         figure();
%         show(eva,configSoln);
%         hold on;
%         focalVector = plot3([eeCoords(1) cFocal(1)],[eeCoords(2) cFocal(2)],[eeCoords(3) cFocal(3)],'k','LineWidth',1.5);
%         object = mesh(X+obscure(j,1),Y+obscure(j,2),Z+obscure(j,3),CO);
%         cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
%         handData = plot3(obscure(:,1), obscure(:,2), obscure(:,3),'b','LineWidth',1.5);
%         startData = plot3(obscure(1,1),obscure(1,2),obscure(1,3),'b^','MarkerSize',10);
%         endData = plot3(obscure(end,1),obscure(end,2),obscure(end,3),'b*','MarkerSize',10);
%         legend([startData,endData,handData,object,cF,focalVector],{'Start of object movement','End of object movement','Object movement path','object','Focal point','Focal vector'},'Location','northwest');
%         xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
%         xlim auto
%         ylim auto
%         zlim auto
%         view(-135,25);
%     end
% end

[rFocal, theta, phi, eeCoords] = calcEndEff(cFocal, rFocal);
[configSoln,currentAngles] = evaIKSolnMATLAB(eeCoords, theta, phi, hAngles);
sideSide = sideSide + [0, 0, -0.5];
[rows,~] = size(sideSide);

for j = 1:rows
    discriminant = lineSphereDiscriminant(eeCoords,cFocal,sideSide(j,:),handRadius);
    if discriminant >= 0
        minTheta = inf;
        
        for k = 1:r
            discriminant = lineSphereDiscriminant(possiblePoints(k,:),cFocal,sideSide(j,:),handRadius);
            if discriminant < 0
                [theta,~] = calcThetaPhi(possiblePoints(k,:),cFocal,rFocal);

                if theta < minTheta
                    minTheta = theta;
                    idx = k;
                end
            end
        end
        
        figure();
        show(eva,configSoln);
        hold on;
        focalVector = plot3([eeCoords(1) cFocal(1)],[eeCoords(2) cFocal(2)],[eeCoords(3) cFocal(3)],'k','LineWidth',1.5);
        object = mesh(X+sideSide(j,1),Y+sideSide(j,2),Z+sideSide(j,3),CO);
        cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
%         plot3(sideSide(:,1), sideSide(:,2), sideSide(:,3),'LineWidth',1.5);
        handData = plot3(sideSide(:,1),sideSide(:,2),sideSide(:,3),'b','LineWidth',1.5);
        startData = plot3(sideSide(1,1),sideSide(1,2),sideSide(1,3),'b^','MarkerSize',10);
        endData = plot3(sideSide(end,1),sideSide(end,2),sideSide(end,3),'b*','MarkerSize',10);
        xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
        legend([startData,endData,handData,object,cF,focalVector],{'Start of object movement','End of object movement','Object movement path','object','Focal point','Focal vector'},'Location','northwest');
        xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
        xlim auto
        ylim auto
        zlim auto
        view(-90,25);
        
        eeCoords = possiblePoints(idx,:);
        [theta,phi] = calcThetaPhi(eeCoords,cFocal,rFocal);
        [configSoln,currentAngles] = evaIKSolnMATLAB(eeCoords,theta,phi,currentAngles);
        
        figure();
        show(eva,configSoln);
        hold on;
        focalVector = plot3([eeCoords(1) cFocal(1)],[eeCoords(2) cFocal(2)],[eeCoords(3) cFocal(3)],'k','LineWidth',1.5);
        object = mesh(X+sideSide(j,1),Y+sideSide(j,2),Z+sideSide(j,3),CO);
        cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
        handData = plot3(sideSide(:,1),sideSide(:,2),sideSide(:,3),'b','LineWidth',1.5);
        startData = plot3(sideSide(1,1),sideSide(1,2),sideSide(1,3),'b^','MarkerSize',10);
        endData = plot3(sideSide(end,1),sideSide(end,2),sideSide(end,3),'b*','MarkerSize',10);
        legend([startData,endData,handData,object,cF,focalVector],{'Start of object movement','End of object movement','Object movement path','object','Focal point','Focal vector'},'Location','northwest');
        xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
        xlim auto
        ylim auto
        zlim auto
        view(-90,25);
    end
end

% plot3(allEE(:,1), allEE(:,2), allEE(:,3), 'x');

% %%
% figure();
% show(eva,configSoln);
% hold on; 
% focalVector = plot3([eeCoords(1) cFocal(1)],[eeCoords(2) cFocal(2)],[eeCoords(3) cFocal(3)],'k','LineWidth',1.5);
% object = mesh(X+obscure(40000,1),Y+obscure(40000,2),Z+obscure(40000,3),CO);
% cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
% arch = plot3(circleInWSpace(:,1),circleInWSpace(:,2),circleInWSpace(:,3),'LineWidth',1.5,'Color','#EDB120');
% circle = plot3(interCircle(:,1),interCircle(:,2),interCircle(:,3),'LineWidth',1.5,'Color','#EDB120');
% hold off;
% legend([object,focalVector,cF,arch],{'Hand object','Focal vector','Focal point','Possible end effector positions'},'location','northwest');
% xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
% title('Obstructing the focal point');
% xlim auto
% ylim auto
% zlim auto
% %%
% possiblePositions = vertcat(circleInWSpace,interCircle);
% 
% [rows,~] = size(possiblePositions);
% c = obscure(40000,:);
% r = handRadius;
% minTheta = inf;
% 
% for j = 1:rows
%     o = possiblePositions(j,:);
%     dHat = (cFocal - o)/norm(cFocal - o);
%     
%     discrim = (dot(dHat,(o - c))^2 - ((norm(o - c))^2 - r^2));
%     
%     if discrim < 0
%         [theta,~] = calcThetaPhi(o,cFocal,rFocal);
%         if theta < minTheta
%             minTheta = theta;
%             idx = j;
%         end
%     end
% end
% 
% bestPos = possiblePositions(idx,:);
% [theta,phi] = calcThetaPhi(bestPos,cFocal,rFocal);
% 
% [configSoln,~] = evaIKSolnMATLAB(bestPos,theta,phi,hAngles);
% 
% %%
% figure();
% 
% show(eva,configSoln);
% hold on; 
% focalVector = plot3([bestPos(1) cFocal(1)],[bestPos(2) cFocal(2)],[bestPos(3) cFocal(3)],'k','LineWidth',1.5);
% object = mesh(X+obscure(40000,1),Y+obscure(40000,2),Z+obscure(40000,3),CO);
% cF = plot3(cFocal(1),cFocal(2),cFocal(3),'x','Color',[0.85 0.325 0.098],'MarkerSize',10);
% arch = plot3(circleInWSpace(:,1),circleInWSpace(:,2),circleInWSpace(:,3),'LineWidth',1.5,'Color','#EDB120');
% circle = plot3(interCircle(:,1),interCircle(:,2),interCircle(:,3),'LineWidth',1.5,'Color','#EDB120');
% hold off;
% legend([object,focalVector,cF,arch],{'Hand object','Focal vector','Focal point','Possible end effector positions'},'location','northwest');
% xlabel('x /meters'); ylabel('y /meters'); zlabel('z /meters');
% title('No longer obstructing the focal point');
% xlim auto
% ylim auto
% zlim auto
        
    


