function [theta, phi] = calcThetaPhi(eeCoords, fCoords, fRadius)
%calcThetaPhi calculates theta and phi 

    noTran = eeCoords - fCoords; % Remove translation so sphere is centred on (0,0,0)
    
    % acos() only returns real numbers if within interval [-1 1]
    if noTran(3) > 1
        noTran(3) = 1;
    end
    
    theta = acos(noTran(3)/fRadius);   % theta = arccos(z/r)
    phi = atan2(noTran(2), noTran(1)); % phi = arctan(y,x)
end
