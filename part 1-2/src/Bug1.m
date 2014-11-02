% Erion Plaku
% Introduction to Robotics
% Department of Electrical Engineering and Computer Science
% Catholic University of America
%
% http://faculty.cua.edu/plaku/
%
% Copyright 2012 by Erion Plaku
% The content may be reproduced under the obligation to acknowledge the source
%
% Modified 2014 by Kenneth Marino
%
function [move, params] = Bug1(~, robotCenter, goalCenter, params, sensor, varargin)
    
    % First input is robotInit, not used by Bug1

    % varargin contains obstacle information
    % Not used directly - passed to CircumnavigateObstacle
    
    % First determine what mode we are in to determine what to do
    if strcmp(params.mode, 'Straight')
        params.color = 'r';
        
        % If not close to obstacle (and we're not at the leave point)
        if ((sensor.dmin > params.step) || ...
                ArePointsNear([sensor.xmin sensor.ymin], params.leave, params.step))
            % Move straight to goal
            move = MakeStepOnLine(robotCenter, goalCenter, params.step);
       
        % If close to obstacle
        else
            % Set to Circle mode
            params.mode = 'Circle';
            
            % Make position the closest point
            move(1) = sensor.xmin - robotCenter(1);
            move(2) = sensor.ymin - robotCenter(2);
            
            % Record hit point
            params.hit(1) = sensor.xmin;
            params.hit(2) = sensor.ymin;
        end            
    elseif strcmp(params.mode, 'Circle')
        params.color = 'r';
        
        % If near hit point
        if (ArePointsNear(robotCenter, params.hit, params.step*0.9) && ...
                params.distLeaveToGoal < inf)
            % Switch mode to Return2Leave
            params.mode = 'Return2Leave';
            
            % Move to hit point
            move(1) = params.hit(1) - robotCenter(1);
            move(2) = params.hit(2) - robotCenter(2);
                        
        % Otherwise
        else            
            % Circumnavigate to next point
            move = CircumnavigateObstacle(robotCenter, params, varargin{:});
            
            % Update leave point if we are closer
            newpos = [0, 0];
            newpos(1) = robotCenter(1) + move(1);
            newpos(2) = robotCenter(2) + move(2);
            distToGoal = norm(newpos - goalCenter);
            if distToGoal < params.distLeaveToGoal
                params.distLeaveToGoal = distToGoal;
                params.leave = newpos;
            end
        end
    elseif strcmp(params.mode, 'Return2Leave')
        params.color = 'g';
        
        % If near leave point        
        if ArePointsNear(robotCenter, params.leave, params.step)
            % Switch mode to Straight
            params.mode = 'Straight';
            
            % Reset dist leave to goal
            params.distLeaveToGoal = inf;
            
        	% Move to leave point
            move(1) = params.leave(1) - robotCenter(1);
            move(2) = params.leave(2) - robotCenter(2);

        % Otherwise
        else
            % Circumnavigate to next point
            move = CircumnavigateObstacle(robotCenter, params, varargin{:});
        end
    end
end
