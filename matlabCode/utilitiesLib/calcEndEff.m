function [rFocal, theta, phi, eeCoords] = calcEndEff(fCoords, fRadius)
%calcEndEff Caclulates endEff coords and orientation
%   Three cases when assuming end effector is directly above focal point at a distance 'r': 
%       - End effector location is within dextrous sphere of robot.
%       - End effector location is outside of dextrous sphere but focal sphere intersects.
%       - End effector location is outside of dextrous sphere and focal sphere doesn't intersect.
%   Function calculates end effector location and orientation so always pointed at focal point at distance 'r' away from it.
    
    cRobot = [0.0, 0.0, 0.187 + 0.096]; % Centre of workspace. From second joint
    rRobot = 0.6 - 0.104; % Radius of robot workspace
    
    cFocal = fCoords; % Centre of focal sphere
    rFocal = fRadius; % Radius of focal sphere
    
    d = norm(cFocal - cRobot); % Distance between centre of both spheres
    
    eeCoords = cFocal + [0.0, 0.0, rFocal]; % Assume directly above
    
    % First case 
    if norm(eeCoords - cRobot) < rRobot
        [theta, phi] = deal(0.0);
    
    % Second case
    elseif d < (rRobot + rFocal) && norm(eeCoords - cRobot) > rRobot
        alpha = 0.5 + ((rRobot^2 - rFocal^2)/(2*(d^2)));
        
        ci = cRobot + alpha*(cFocal - cRobot); % Centre of intersect circle
        
        ri = sqrt(rRobot^2 - (alpha*d)^2); % Radius of intersect circle
        
        [~, normU, V] = calcTangentBitangent(cRobot, cFocal);
        
        [~, eeCoords, ~] = calcTanBitanPointsAndVector(ci, normU, V, ri, 'zMax');
        
        [theta, phi] = calcThetaPhi(eeCoords, cFocal, rFocal); 
    
    % Third case
    elseif d >= (rRobot + rFocal)
        if d > (rRobot + rFocal)
            rFocal = d - rRobot;
        end
        
        eeCoords = cRobot + rRobot*(cFocal - cRobot);
        
        [theta, phi] = calcThetaPhi(eeCoords, cFocal, rFocal);
    end                     
end 