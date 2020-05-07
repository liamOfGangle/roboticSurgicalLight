function [discriminant] = lineSphereDiscriminant(lineOrigin,lineEnd,sphereCentre,sphereRadius)
%lineSphereDiscriminant calculates line - sphere discriminant

    dHat = (lineEnd - lineOrigin)/norm(lineEnd - lineOrigin);
    discriminant = (dot(dHat,(lineOrigin - sphereCentre))^2 - ((norm(lineOrigin - sphereCentre))^2 - sphereRadius^2));
end

