function [closestAxis] = dominantAxis(pointA,pointB,angleOfIncidence)
%dominantAxis calculates if the incoming object is traveling along basis
%axes
%   Calculates if  the incoming object is traveling along one of the basis
%   axes within a certain angle, e.g. object is travelling along 30 degrees
%   to x axis
    
    standardBasis = [1 0 0; 
                     0 1 0; 
                     0 0 1;
                     -1 0 0; 
                     0 -1 0; 
                     0 0 -1];
    
    standardBasisNames = ["x positive","y positive","z positive", ...
                          "x negative","y negative","z negative"];          
       
    vector = pointB - pointA;
    normVec = norm(vector);
    
    closestAxis = 'No closest axis';
    
    [rows,~] = size(standardBasis);
    
    for j = 1:rows
        cosAlpha = dot(vector,standardBasis(j,:))/normVec;
        alpha = acosd(cosAlpha);
        if alpha <= angleOfIncidence
            closestAxis = standardBasisNames(j);
        end
    end 
    
    if strcmp(closestAxis,"x positve") || strcmp(closestAxis,"x negative")
        closestAxis = "x axis";
    elseif strcmp(closestAxis,"y positve") || strcmp(closestAxis,"y negative")
        closestAxis = "y axis";
    elseif strcmp(closestAxis,"z positve") || strcmp(closestAxis,"z negative")
        closestAxis = "z axis";
    end
end

