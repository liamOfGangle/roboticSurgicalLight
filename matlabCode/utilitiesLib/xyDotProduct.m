function [cosAngle] = xyDotProduct(vector)
%xyDDotProduct calculates 2d dot product in xy of vector and x axis
%   cos(angle) = A.B/(||A||*||B||), B = [1 0]. Returns cos(angle)

    A = [vector(1) vector(2)];
    B = [1 0];
    
    cosAngle = dot(A,B)/norm(A);
end

