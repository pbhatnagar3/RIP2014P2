function moves = RRT(obstacles)
    obj = VideoWriter('rectOb1');
    open(obj);
    thetas = [-.5, 0, 0];
    maxIter = 300;
    edgeMat = zeros(maxIter,maxIter);
    goalFound = false;
    thresh = .01;
     armLengths = [2,2,1];
     startNode = getEndPosition(thetas, armLengths)'; 
     figure;
    
     thetaTree = thetas';
     tree = startNode;
     plot(startNode(1), startNode(2), 'g.', 'MarkerSize', 20);
     h1 = gcf;
     hold on;
     %plot(endNode(1), endNode(2), 'm.', 'MarkerSize', 20);
     axis square
     axis equal
     xlabel 'x-distance';
     ylabel 'y-distance';
     [row,~] = size(obstacles);
     circles = [];
     for r = 1:row
         obs = obstacles(r,:);
         corners = [-1.*obs(4), obs(4), obs(4), -1.*obs(4), -1.*obs(4);...
             -1.*obs(5), -1.*obs(5), obs(5), obs(5), -1.*obs(5)];
         rot = [cos(obs(3)), -1.*sin(obs(3)); sin(obs(3)), cos(obs(3))];
         corners = rot*corners;
         corners(1,:) = corners(1,:)./2 + obs(1);
         corners(2,:) = corners(2,:)./2 + obs(2);
        plot(corners(1,:), corners(2,:), 'g-');
        circles = [circles; corners];
     end
   baseTheta = thetaTree(:, 1);
   count = 0;
   numIter = 0;
   while numIter < maxIter

        newTh1 = mod(baseTheta(1)-rand()*pi/2 + pi/4, 2*pi);
        newTh2 = mod(baseTheta(2)+pi -rand()*pi/2 + pi/4, 2*pi) - pi; 
        newTh3 = mod(baseTheta(3)+pi -rand()*pi/2+ pi/4, 2*pi) - pi; 
        theta = [newTh1, newTh2, newTh3];
        newNode = getEndPosition(theta, armLengths)';
        distanceVec =(thetaTree(1,:) - theta(1)).^2+ (thetaTree(2,:)-theta(2)).^2+(thetaTree(3,:)-theta(3)).^2;
        
       [val, ind] = min(distanceVec);
       nearestTheta = thetaTree(:,ind);
       temp = theta' - nearestTheta;
       thetaVec = temp./(sqrt(sum(temp.^2))*10);
       theta = (nearestTheta + thetaVec)';
       newNode = getEndPosition(theta, armLengths)';
       nearestNode = tree(:,ind);
       count = count +1;
       if ~getColisions(theta, circles)
           numIter = numIter +1;
                tree = [tree, newNode];
                thetaTree = [thetaTree, theta'];
                edgeMat(ind, length(tree)) = val;
                edgeMat(length(tree), ind) = val;
                 plot([nearestNode(1), newNode(1)], [nearestNode(2), newNode(2)], 'ko-');    
%            current_thetas = theta;
%            %finding the coordinates of joint1 
%             joint1_x = armLengths(1) * cos(current_thetas(1));
%             joint1_y = armLengths(1) * sin(current_thetas(1));
% 
%             %finding the coordinates of joint2
%             joint2_x = joint1_x + armLengths(2) * cos(current_thetas(1) + current_thetas(2));
%             joint2_y = joint1_y + armLengths(2) * sin(current_thetas(1) + current_thetas(2));
% 
%             %finding the location of endPoint
%             endPosition_x = joint2_x + armLengths(3) * cos(current_thetas(1) + current_thetas(2) + current_thetas(3));
%             endPosition_y = joint2_y + armLengths(3) * sin(current_thetas(1) + current_thetas(2) + current_thetas(3));
% 
%              plot([0.0 ; joint1_x], [0.0 ; joint1_y], 'o-'); 
%              h2 = gcf;
%              plot([joint1_x ; joint2_x], [joint1_y ; joint2_y], 'ro-');
%              plot([joint2_x ; endPosition_x], [joint2_y ; endPosition_y], 'bo-'); 
                writeVideo(obj, getframe(gcf));
       end   
       if count >=10
           [rr, cc] = size(thetaTree);
           newInd = ceil(rand()*(cc-1)+1);
           baseTheta = thetaTree(:, newInd);
           count = 0;
       end
   end
end

function isIntersect = getColisions(endTh, obstacles)
    endCoords = getEndPositions(endTh, [2,2,1]);
    endCoords = [[0;0], endCoords];
    
    isIntersect = false;
    [row, ~] = size(obstacles);
    for r = 1:2:row
        obs = obstacles(r:r+1, :);
        for np = 1:3
            xy1 = endCoords(:,np:np+1);
            for nobs = 1:4
                if ~isIntersect
                    xy2 = obs(:, nobs:nobs+1);
                    isIntersect =  getIntersect(xy1, xy2);
                end
            end
        end
    end

end

function dist = getDist(p1, p2, p3)
    diffVec = p2-p1;
    something = sum(diffVec.^2);
    u = sum((p3-p1).*diffVec)/something;
    if u>1
        u=1;
    elseif u<0
        u=0;
    end
    
    xy = p1 + u.*diffVec;
    dxy = xy - p3;
    dist = sqrt(sum(dxy.^2));
end

function boolVal = getIntersect(vec1, vec2)
    x = [vec1(1,:)', vec2(1,:)'];%# Starting points in first row, ending points in second row
    y = [vec1(2,:)', vec2(2,:)'];
    dx = diff(x);  %# Take the differences down each column
    dy = diff(y);
    den = dx(1)*dy(2)-dy(1)*dx(2);  %# Precompute the denominator
    ua = (dx(2)*(y(1)-y(3))-dy(2)*(x(1)-x(3)))/den;
    ub = (dx(1)*(y(1)-y(3))-dy(1)*(x(1)-x(3)))/den;

    boolVal = all(([ua ub] >= 0) & ([ua ub] <= 1));

end



