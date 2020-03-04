function [newJointAngles] = ikSolution(focalPos,focalOri,delta)

    load 'eva.mat';

    ee = 'endEffector';
    bl = 'baseLink';

    focalPos(3) = focalPos(3) + delta;
    focalPos = trvec2tform(focalPos);

    focalOri = axang2tform(focalOri);

    tform = focalPos*focalOri;

    q0 = homeConfig;
    [configSoln, solnInfo] = ik(ee,tform,weights,q0);
    
    newJointAngles = zeros(size(configSoln));
    
    for i = 1:length(configSoln)
        newJointAngles(i) = configSoln(i).JointPosition;
    end
    
    show(eva, homeConfig);
    hold on
    show(eva, configSoln);
    
    t0 = getTransform(eva, homeConfig, ee);
    tEnd = tform;
    tInterval = [0 1];
    tvec = (0:0.01:1);
    
    [tfInterp, v1, a1] = transformtraj(t0,tEnd,tInterval,tvec);
    rotations = tform2quat(tfInterp);
    translations = tform2trvec(tfInterp);

    plotTransforms(translations,rotations)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    hold off
end

