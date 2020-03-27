function [rFocal, theta, phi, eeCoords] = calcEndEff(fCoords, fRadius)
%calcEndEff Caclulates endEff coords and orientation
%   Three cases when assuming end effector is directly above focal point at a distance 'r': 
%       - End effector location is within dextrous sphere of robot.
%       - End effector location is outside of dextrous sphere but focal sphere intersects.
%       - End effector location is outside of dextrous sphere and focal sphere doesn't intersect.
%   Function calculates end effector location and orientation so always pointed at focal point at distance 'r' away from it.
 
    [theta, phi] = deal(0.0);
    
    cRobot = [0.0, 0.0, 0.187 + 0.096]; % Centre of workspace. From second joint
    rRobot = 0.6 - 0.104; % Radius of robot workspace
    
    cFocal = fCoords; % Centre of focal sphere
    rFocal = fRadius; % Radius of focal sphere
    
    d = norm(cFocal - cRobot); % Distance between centre of both spheres
    
    eeCoords = cFocal + [0.0, 0.0, rFocal]; % Assume directly above
    
    % First case iff following statements aren't true
    
    % Second case
    if d < (rRobot + rFocal) && norm(eeCoords - cRobot) > rRobot
        rho = linspace(0, 359, 360);
        rho = deg2rad(rho);
        
        alpha = 0.5 + ((rRobot^2 - rFocal^2)/(2*(d^2)));
        
        ci = cRobot + alpha(cFocal - cRobot); % Centre of intersect circle
        
        ri = sqrt(rRobot^2 - (alpha*d)^2); % Radius of intersect circle
        
        normCC = (cFocal - cRobot)/d; % Normalised vector that runs perpendicular to intersect circle
        
        xu = (-1.0*(normCC(3) + normCC(2)))/normCC(1); % x value of tangent
        yu = (-1.0*(normCC(3) + normCC(1)))/normCC(2); % y value of tangent
        zu = (-1.0*(normCC(2) + normCC(1)))/normCC(3); % z value of tangent
        
        U = [xu, 1, 1]; % Tangent vector U
        normU = U/norm(U); % Normalised U vector
        
        if any(normU > 1.0)
            U = [1, yu, 1];
            normU = U/norm(U);
            
            if any(normU > 1.0)
                U = [1, 1, zu];
                normU = U/norm(U);
                
                if any(normU > 1.0)
                    error("Element/s of normU are >1")
                    return
                end
            end
        end
        
        V = cross(normCC, normU); % Bitangent vector
        
        zMax = -inf; % Choose max z so angle between (ee - fc) & [0 0 1] is a minimum
        
        for j = 1:length(rho)
            P = ci + ri*(normU*cos(rho(j)) + V*sin(rho(j))); % Intersect point P(rho) on circumference of intersect circle
            if P(3) > zMax
                zMax = P(3);
                eeCoords = P;
            end
        end
        
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

