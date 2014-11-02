% Kenneth Marino 2014
%
function [U, dU] = CalculateAttractivePotential(robotCenter, goalCenter, params)
    
    % Get distance to goal
    dist2goal = norm(robotCenter - goalCenter);
    
    % Set potential
    if (dist2goal <= params.goalThresh)
        U = 0.5 * params.attrScale * dist2goal^2;
    else
        U = params.goalThresh * params.attrScale * dist2goal - ...
            0.5 * params.attrScale * params.goalThresh^2;
    end
    
    % Set gradient potential
    if (dist2goal <= params.goalThresh)
        dU = params.attrScale * (robotCenter - goalCenter);
    else
        dU = params.goalThresh * params.attrScale * ...
            (robotCenter - goalCenter) / dist2goal;
    end
end