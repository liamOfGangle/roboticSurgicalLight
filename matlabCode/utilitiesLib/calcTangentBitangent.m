function [normVec,normTangent,normBitangent] = calcTangentBitangent(pointA,pointB)
%calcTangentBitangent calculates vector, tangent and bitangent
%   Calculates unit vector between A & B, unit tangent and bitangent vector
%   to the AB vector
  
    normVec = (pointB - pointA)/norm(pointB - pointA);
    
    xTangent = (-1.0*(normVec(3) + normVec(2)))/normVec(1); % x value
    yTangent = (-1.0*(normVec(3) + normVec(1)))/normVec(2); % y value
    zTangent = (-1.0*(normVec(2) + normVec(1)))/normVec(3); % z value
    
    tangent = [xTangent, 1, 1]; % Tangent vector
    normTangent = tangent/norm(tangent); % Unit tangent
    
    if any(normTangent > 1.0)
    tangent = [1, yTangent, 1];
    normTangent = tangent/norm(tangent);

        if any(normTangent > 1.0)
            tangent = [1, 1, zTangent];
            normTangent = tangent/norm(tangent);

            if any(normTangent > 1.0)
                error('Element/s of normU are >1')
            end
        end
    end
    
    normBitangent = cross(normVec,normTangent); % Unit bitangent vector
end

