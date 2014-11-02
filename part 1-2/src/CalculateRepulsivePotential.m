% Kenneth Marino 2014
%
function [U, dU] = CalculateRepulsivePotential(robotCenter, sensors, params)
    % Set potentials (initially 0)
    U = 0;
    dU = [0, 0];
    
    for sensor = sensors
        % Set potential
        if (sensor.dmin <= params.obsThresh)
            U = U + 0.5 * params.repulScale * (1/sensor.dmin - 1/params.obsThresh)^2;
        end

        % Calculate differential (direction) from nearest obstacle
        dD = (robotCenter - [sensor.xmin sensor.ymin]) / ...
            norm(robotCenter - [sensor.xmin, sensor.ymin]);

        % Set gradient potential
        if (sensor.dmin <= params.obsThresh)
            dU = dU + params.repulScale * (1/params.obsThresh - 1/sensor.dmin) * ...
                1/(sensor.dmin^2) * dD;
        end
    end
end