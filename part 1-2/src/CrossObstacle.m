% Kenneth Marino 2014
%
function cross = CrossObstacle(robotCenter, goalCenter, params, varargin)
    % Determine if following the m-line will interesect an obstacle
    
    % Take one step on m-line
    xdist = goalCenter(1) - robotCenter(1);
    ydist = goalCenter(2) - robotCenter(2);
    disttot = norm([xdist, ydist]);
    move = [0, 0];
    move(1) = (xdist / disttot) * params.step;
    move(2) = (ydist / disttot) * params.step;
    newloc = [0, 0];
    newloc(1) = robotCenter(1) + move(1);
    newloc(2) = robotCenter(2) + move(2);

    % Switch based on domain type
    if strcmp(params.domaintype, 'vertices')
        % Get obstacle data
        xcoordinates = varargin{1};
        ycoordinates = varargin{2};
        
        % If the new point is in the obstacle, then following the m line
        % will cause it to cross
        in = inpolygon(newloc(1), newloc(2), xcoordinates, ycoordinates);
        if in
            cross = 1;
        else
            cross = 0;
        end
        
    elseif strcmp(params.domaintype, 'circles')
        % Get obstacle data
        centerx = varargin{1};
        centery = varargin{2};
        radius = varargin{3};
        
        cross = 0;
        
        % Even simpler, check if it's within rad of each obstacle
        for i = 1:length(centerx)
            x = centerx(i);
            y = centery(i);
          	rad = radius(i);
            
            if norm([x, y] - robotCenter) < rad
                cross = 1;
            end
        end
    end
end