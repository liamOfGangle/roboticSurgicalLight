function [discriminant] = obscureCheck(focalPointCoordinates, focalDistance, theta, phi, obstacleRadius, obstacleCentreCoordinates)
%obscureCheck Checks for intersects with object
%   Calculates if endEffector to focalPoint vector intersects a spherical object.
%   Uses formula found here - https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection

    originPoint = zeros(1,3);
    originPoint(1) = focalPointCoordinates(1) + focalDistance*sin(theta)*cos(phi);
    originPoint(2) = focalPointCoordinates(2) + focalDistance*sin(theta)*sin(phi);
    originPoint(3) = focalPointCoordinates(3) + focalDistance*cos(theta);
    
    unitVec = norm(focalPointCoordinates - originPoint);
    
    discriminant = dot(unitVec,(originPoint - obstacleCentreCoordinates))^2 - (norm(originPoint - obstacleCentreCoordinates)^2 - obstacleRadius^2);
    
    if discriminant >= 0
        discriminant = 1;
    else
        discriminant = 0;
    end
end

