function [theta, phi] = calcThetaPhi(eeCoords, fCoords, fRadius)
%calcThetaPhi calculates theta and phi 

    noTran = eeCoords - fCoords; % Remove translation so sphere is centred on (0,0,0)
    
    theta = acos(noTran(3)/fRadius);   % theta = arccos(z/r)
    phi = atan2(noTran(2), noTran(1)); % phi = arctan(y,x)
end
