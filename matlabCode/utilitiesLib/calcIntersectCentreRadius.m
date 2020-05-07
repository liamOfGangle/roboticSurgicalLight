function [intersectCentre,intersectRadius] = calcIntersectCentreRadius(sphereA,sphereB,radiusA,radiusB)
%calcIntersectCentreRadius Calculates radius and centre of intersection
%circle of two spheres
    
%     sphereA, radiusA = focal
%     sphereB, radiusB = robot
    
    d = norm(sphereA - sphereB);
    alpha = 0.5 + ((radiusB^2 - radiusA^2)/(2*(d^2)));
    intersectCentre = sphereB + alpha*(sphereA - sphereB); % Centre of intersect circle
    intersectRadius = sqrt(radiusB^2 - (alpha*d)^2); % Radius of intersect circle
end

