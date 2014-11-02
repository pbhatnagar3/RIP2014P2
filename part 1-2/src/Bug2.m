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
function [move, params] = Bug2(robotInit, robotCenter, goalCenter, params, sensor, varargin)
    
    % varargin contains obstacle information
    % Not used directly - passed to CircumnavigateObstacle
    
    % First determine what mode we are in to determine what to do
    if strcmp(params.mode, 'Straight')
        % If not close to obstacle (and we're not at the leave point)
        if ((sensor.dmin > params.step) || ...
                ArePointsNear([sensor.xmin, sensor.ymin], params.leave, params.step))
            % Move straight to goal
            move = MakeStepOnLine(robotCenter, goalCenter, params.step);
       
        % If close to obstacle
        else
            % Flip color assignment
            if strcmp(params.color, 'r')
                params.color = 'g';
            else
                params.color = 'r';
            end
            
            % Set to Circle mode
            params.mode = 'Circle';
            
            % Make position the closest point
            move(1) = sensor.xmin - robotCenter(1);
            move(2) = sensor.ymin - robotCenter(2);
            
            % Record hit point
            params.hit(1) = sensor.xmin;
            params.hit(2) = sensor.ymin;
            params.distLeaveToGoal = ...
                norm([sensor.xmin, sensor.ymin] - goalCenter);
            
        end            
    elseif strcmp(params.mode, 'Circle')
        % If near m-line
        if IsPointNearLine(robotCenter, robotInit, goalCenter) && ...
                ~ArePointsNear(params.hit, robotCenter, params.step) && ...
                norm(robotCenter - goalCenter) < params.distLeaveToGoal && ...
                ~CrossObstacle(robotCenter, goalCenter, params, varargin{:})
            
            % Switch mode to Straight
            params.mode = 'Straight';
             
            % Move to m-line
            [newloc, ~] = ProjectPoint([robotInit; goalCenter], robotCenter);
            move(1) = newloc(1) - robotCenter(1);
            move(2) = newloc(2) - robotCenter(2);
            
            % Set leave point
            params.leave = newloc;
           
        % Otherwise
        else
            % Circumnavigate to next point
            move = CircumnavigateObstacle(robotCenter, params, varargin{:});
        end
    end
end


