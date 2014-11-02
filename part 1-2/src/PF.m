% Created 2014 by Kenneth Marino
%
function [move, params] = PF(robotInit, robotCenter, goalCenter, params, sensor, varargin)
    
  % For now, only works with circles
   if strcmp(params.domaintype, 'circles')
        % Get obstacle data
        centerx = varargin{1};
        centery = varargin{2};
        radius = varargin{3};
    end

    % Get sensors (all obstacles now)
    sensors = TakeSensorReadingAllObs(params.domaintype, robotCenter, varargin{:});
    
    % Get attractive potential
    [U_a, dU_a] = CalculateAttractivePotential(robotCenter, goalCenter, params);
    
    % Get repulsive potential
    [U_r, dU_r] = CalculateRepulsivePotential(robotCenter, sensors, params);

    % Sum together
    U = U_a + U_r;
    dU = dU_a + dU_r;
    
    % Move is alpha times dU
    move = -params.alpha * dU;
    
end
