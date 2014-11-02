% Kenneth Marino 2014
%
function move = CircumnavigateObstacle(robotCenter, params, varargin)
    % Determine how we should move to circumnavigate the obstacle
    % Note: always going to move CW around obstacle

    % Switch based on domain type
    if strcmp(params.domaintype, 'vertices')
        % Get obstacle data
        xcoordinates = varargin{1};
        ycoordinates = varargin{2};
        
        % Determine line segment we are on
        startVertexInd = 0;
        endVertexInd = 1;
        startVertex = [0, 0];
        endVertex = [0, 0];
        for vertex = 1:length(xcoordinates)
            startVertexInd = vertex;
            endVertexInd = startVertexInd + 1;
            startVertex = [xcoordinates(startVertexInd), ...
                ycoordinates(startVertexInd)];
            endVertex = [xcoordinates(endVertexInd), ...
                ycoordinates(endVertexInd)];
            
            % If already on a vertex, it's on the end vertex
            if (endVertex(1) == robotCenter(1) && ...
                    endVertex(2) == robotCenter(2))
                if endVertexInd == length(xcoordinates)
                    startVertexInd = 0;
                else
                    startVertexInd = endVertexInd;
                end
                endVertexInd = startVertexInd + 1;
                startVertex = [xcoordinates(startVertexInd), ...
                    ycoordinates(startVertexInd)];
                endVertex = [xcoordinates(endVertexInd), ...
                    ycoordinates(endVertexInd)];
                break;
            % If we are on the current segment, break
            elseif IsPointNearLine(robotCenter, startVertex, endVertex) && ...
                    IsPointInSegment(robotCenter, startVertex, endVertex)
                break;
            end
        end
        
        % Now move along that line
        move = MakeStepOnLine(robotCenter, endVertex, params.step);
        
    elseif strcmp(params.domaintype, 'circles')
        % Get obstacle data
        centerx = varargin{1};
        centery = varargin{2};
        radius = varargin{3};
        
        % Determine which circle we are on
        circleind = 1;
        for i = 1:length(centerx)
            x = centerx(i);
            y = centery(i);
            rad = radius(i);
            
            if norm([x, y] - robotCenter) < (rad + 0.064)
                circleind = i;
                break;
            end
        end
        
        % Get circle info
        cx = centerx(circleind);
        cy = centery(circleind);
        rad = radius(circleind);
        
        % Get the theta that we are on for the circle
        ang = atan2(robotCenter(2) - cy, robotCenter(1) - cx);
                
        % Determine theta difference we should set
        thetadelta = params.step / rad;
        angnew = ang - thetadelta;
        if angnew < -pi
            angnew = angnew + 2*pi;
        end
        
        % Get corresponding x, y location
        newx = rad*cos(angnew) + cx;
        newy = rad*sin(angnew) + cy;
        
        % Now determine move
        move = [0, 0];
        move(1) = newx - robotCenter(1);
        move(2) = newy - robotCenter(2);
    end
end