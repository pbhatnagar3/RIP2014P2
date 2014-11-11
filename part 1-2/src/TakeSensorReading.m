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
function sensor = TakeSensorReading(domaintype, robotCenter, varargin)
    % Take sensor reading from robot center
    %   sensor.dmin : minimum distance from robot center to obstacle boundary
    %   sensor.xmin : x-coordinate of obstacle boundary point that is closest to robot center
    %   sensor.ymin : y-coordinate of obstacle boundary point that is closest to robot center

    % Switch based on domain type
    if strcmp(domaintype, 'vertices')
        % Get obstacle data
        xcoordinates = varargin{1};
        ycoordinates = varargin{2};
        poly = zeros(1,2*length(xcoordinates));
        poly(1:2:end) = xcoordinates;
        poly(2:2:end) = ycoordinates;
        [xmini, ymini, dmini] = PointPolygonDistanceSquare(robotCenter(1), robotCenter(2), poly);
        sensor.xmin = xmini;
        sensor.ymin = ymini;
       	sensor.dmin = sqrt(dmini);
        
    elseif strcmp(domaintype, 'circles')
        % Get obstacle data
        centerx = varargin{1};
        centery = varargin{2};
        radius = varargin{3};
        
        % Check each obstacle
        sensor.dmin = Inf;
        for i = 1:length(centerx)
            x = centerx(i);
            y = centery(i);
          	rad = radius(i);
            
            % Determine closest point on circle to robot position
            [xmini, ymini, dmini] = PointCircleDistanceSquare(robotCenter(1), robotCenter(2), x, y, rad);
            if sqrt(dmini) < sensor.dmin
                sensor.xmin = xmini;
                sensor.ymin = ymini;
                sensor.dmin = sqrt(dmini);
            end
        end
        
    end
end

function [xmin, ymin, dmin] = PointPolygonDistanceSquare(px, py, poly)
    n2 = length(poly);
    [xmin, ymin, dmin] = PointSegmentDistanceSquare(px, py, poly(n2 - 1), poly(n2), poly(1), poly(2));
    for i = 1:2:n2-3
        [xmini, ymini, dmini] = PointSegmentDistanceSquare(px, py, poly(i), poly(i+1), poly(i+2), poly(i+3));
        if (dmini < dmin)
            xmin = xmini;
            ymin = ymini;
            dmin = dmini;
        end
    end       
end

function [xmini, ymini, dmini] = PointCircleDistanceSquare(px, py, cx, cy, rad)
    vx = px - cx;
    vy = py - cy;
    magv = sqrt(vx*vx + vy*vy);
    ax = vx * rad/magv + cx;
    ay = vy * rad/magv + cy;
    xmini = ax;
    ymini = ay;
    dmini = (ax-px)^2 + (ay-py)^2;
end

function [xmin, ymin, dmin] = PointSegmentDistanceSquare(px, py, s0x, s0y, s1x, s1y)
    vx = s1x - s0x;
    vy = s1y - s0y;
    wx = px  - s0x;
    wy = py  - s0y;
    a  = vx * wx + vy * wy;
    b  = vx * vx + vy * vy;
    
    if (a <= 0) 
        xmin = s0x;
        ymin = s0y;
        dmin = wx * wx + wy * wy;
        return;
    end
	
    if (b <= a)
        xmin = s1x;
        ymin = s1y;
        dmin = (px - s1x) * (px - s1x) + (py - s1y) * (py - s1y);
        return;
    end
    
    a = a ./ b;
    vx= s0x + a * vx;
    vy= s0y + a * vy;
    
    xmin = vx;
    ymin = vy;
    dmin = (px - vx) * (px - vx) + (py - vy) * (py - vy);    
end