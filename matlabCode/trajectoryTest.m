clear all, close all
%%
load eva.mat
%% Focal "dimensions"
height = 0.05;
radius = height/2;
focalPos = [-0.3, 0, -height];
%% Add a fixed body to robot model
body = rigidBody('focalFrame');
setFixedTransform(body.Joint, trvec2tform(focalPos))
addBody(eva, body, eva.BaseName);
%%
q0 = zeroConfig;
qHome = homeConfig;
%%
start = 'baseLink';
ee = 'endEffector';
%%
q0Vector = tform2trvec(getTransform(eva, q0, ee, start));
qHomeVector = tform2trvec(getTransform(eva, qHome, ee, start));
%%
x = linspace(q0Vector(1), qHomeVector(1), 10);
y = linspace(q0Vector(2), qHomeVector(2), 10);
z = linspace(q0Vector(3), qHomeVector(3), 10);
%%
angle = linspace(0, -pi, 10);
%%
qInit = q0;
for i = 1:length(x)
    point = [x(i) y(i) z(i)];
    rot = [0 1 0 angle(i)];
    tform = trvec2tform(point)*axang2tform(rot);
    qSol = ik(ee,tform,weights,qInit);
    qs(i,:) = qSol;
    qInit = qSol;
end
%%
figure;
show(eva,qInit);
%%
fps = 1;
r = rateControl(fps);
%%
t0 = getTransform(eva, zeroConfig, 'link2');
tHome = getTransform(eva, homeConfig, 'link2');
tInterval = [0 1];
tvec = (0:0.01:1);
%%
hold on
for i = 1:length(x)
    show(eva,qs(i,:)','PreservePlot',false);
    waitfor(r);
end

%%
[tfInterp, v1, a1] = transformtraj(t0,tHome,tInterval,tvec);
rotations = tform2quat(tfInterp);
translations = tform2trvec(tfInterp);

plotTransforms(translations,rotations)
xlabel('X')
ylabel('Y')
zlabel('Z')