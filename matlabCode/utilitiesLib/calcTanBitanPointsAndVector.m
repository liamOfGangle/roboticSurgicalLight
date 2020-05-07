function [allPoints,desiredPoint,desiredVector] = calcTanBitanPointsAndVector(inputPoint,tangent,bitangent,radius,desired)
%calcTanBitanPointsAndVector calculates all points on circle described by
%tangent and bitangent
%   Calculates all points on circle described by the tangent and
%   bitangent, returns all the points, desired point and desired vector.
%   Desired vector is a unit vector if radius == 1.

    rho = deg2rad(linspace(0,360,360+1));
    allPoints = [];
    
    for j = 1:length(rho)
        pRho = inputPoint + radius*(tangent*cos(rho(j)) + bitangent*sin(rho(j)));
        allPoints = vertcat(allPoints,pRho);
    end
    
    switch desired
        case 'xMax'
            [~,idx] = max(allPoints(:,1));
            desiredPoint = allPoints(idx,:);
            desiredVector = desiredPoint - inputPoint;
        case 'xMin'
            [~,idx] = min(allPoints(:,1));
            desiredPoint = allPoints(idx,:);
            desiredVector = desiredPoint - inputPoint;
        case 'yMax'
            [~,idx] = max(allPoints(:,2));
            desiredPoint = allPoints(idx,:);
            desiredVector = desiredPoint - inputPoint;
        case 'yMin'
            [~,idx] = min(allPoints(:,2));
            desiredPoint = allPoints(idx,:);
            desiredVector = desiredPoint - inputPoint;
        case 'zMax'
            [~,idx] = max(allPoints(:,3));
            desiredPoint = allPoints(idx,:);
            desiredVector = desiredPoint - inputPoint;
        case 'zMin'
            [~,idx] = min(allPoints(:,3));
            desiredPoint = allPoints(idx,:);
            desiredVector = desiredPoint - inputPoint;
        otherwise
            error('Must be a max/min in x, y or z');
    end
end

