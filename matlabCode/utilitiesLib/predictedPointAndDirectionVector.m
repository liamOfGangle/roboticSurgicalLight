function [directionVector,predictedPoint] = predictedPointAndDirectionVector(pointA,pointB)
%predictedPointInSpace calculates predicted point in space from two
%prevoius points that is 2x as far ahead on the direction vector from point
%B
    
    vectorBetween = pointB - pointA;
    
    if all(vectorBetween == 0)
        predictedPoint = pointB;
        directionVector = [0 0 0];
    else
        directionVector = 2*vectorBetween;
        predictedPoint = pointB + directionVector;
    end
end

