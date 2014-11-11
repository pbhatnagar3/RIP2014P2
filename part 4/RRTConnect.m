function moves = RRTConnect(startNode, endNode, obstacles)
    t = cputime;
    obj = VideoWriter('ConnectTree');
    obj2 = VideoWriter('ConnectArm');
    open(obj);
    open(obj2);
    maxIter = 900;
    edgeMat = zeros(maxIter,maxIter);
    goalFound = false;
    thresh = .05;
     armLengths = [2,2,1];
     figure;
     thetas = startNode'
     plot3(thetas(1), thetas(2), thetas(3), 'g.', 'MarkerSize', 20);
     thetaTree = thetas';
    % plot(startNode(1), startNode(2), 'g.', 'MarkerSize', 20);
     hold on;
      plot3(endNode(1), endNode(2), endNode(3), 'm.', 'MarkerSize', 20);
    % plot(endNode(1), endNode(2), 'm.', 'MarkerSize', 20);
     %plot(endNode(1), endNode(2), 'm.', 'MarkerSize', 20);
     axis square
     axis equal
%      xlabel 'x-distance';
%      ylabel 'y-distance';
     %axis([-5,5,-5,5]);
     xlabel 'theta 1';
     ylabel 'theta 2';
     zlabel 'theta 3';
     view(3);
     axis([-pi,pi,-pi,pi, -pi, pi]);
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
       % plot(corners(1,:), corners(2,:), 'g-');
        circles = [circles; corners];
     end
   baseTheta = thetaTree(:, 1);
   count = 0;
   numIter = 0;

   while numIter < maxIter-1 &~goalFound

        newTh1 =  max(min(baseTheta(1) -rand()*pi/2 + pi/4, pi),-pi); 
        newTh2 = max(min(baseTheta(2) -rand()*pi/2 + pi/4, pi),-pi); 
        newTh3 = max(min(baseTheta(3) -rand()*pi/2 + pi/4, pi),-pi); 
        theta = [newTh1, newTh2, newTh3];
        distanceVec =(thetaTree(1,:) - theta(1)).^2+ (thetaTree(2,:)-theta(2)).^2+(thetaTree(3,:)-theta(3)).^2;
        
       [val, ind] = min(distanceVec);
       nearestTh = thetaTree(:,ind);
       temp = theta' - nearestTh;
       amp = sqrt(sum(temp.^2));
       steps = ceil(amp/.25);
       thetaVec = temp./(amp*4);    
       newNode = getEndPosition(theta, armLengths)';
       
       count = count +1;
       theta = nearestTh+ thetaVec;
       for iii = 1:steps
           if ~getColisions(nearestTh, theta, circles) && iii~=steps
               nearestTh = theta;
               theta = theta + thetaVec;
           elseif iii~= 1 | iii == steps
               numIter = numIter +1;
                theta = theta-thetaVec;
               distFromGoal = sum((theta - endNode).^2);
               if distFromGoal < thresh
                   goalFound = true;
               end
                    thetaTree = [thetaTree, theta];
                    edgeMat(ind, length(thetaTree)) = val;
                    edgeMat(length(thetaTree), ind) = val;
                    %plot([nearestNode(1), newNode(1)], [nearestNode(2), newNode(2)], 'ko-');  
                    nearestTh = thetaTree(:,ind);
                    plot3([theta(1) nearestTh(1)], [theta(2) nearestTh(2)], [theta(3) nearestTh(3)], 'ko-', 'MarkerSize', 2);  
                   % writeVideo(obj, getframe(gcf));
           else 
               break;
           end   
           if count >=10
               x = rand();
               if x > .3
                     distanceVec =(thetaTree(1,:) - endNode(1)).^2+ (thetaTree(2,:)...
                         -endNode(2)).^2+(thetaTree(3,:)-endNode(3)).^2;
                   [~, minInd] = min(distanceVec);
               else
                  [~, ccol] = size(thetaTree);
                  minInd = ceil(rand()*ccol);
               end
               baseTheta = thetaTree(:, minInd);
               count = 0;
           end
       end
   end
   curInd = 1;
   newNodes = [1];
   explored = [];
   path = [1];
   c = PriorityQueue();
   c.insert(-1, path);
   curPath = [];
   curDist = inf;
   curThresh = .05;
   notfound = true;
   distanceVec =(thetaTree(1,:) - endNode(1)).^2+ (thetaTree(2,:)...
                 -endNode(2)).^2+(thetaTree(3,:)-endNode(3)).^2;
   [~, minInd] = min(distanceVec);
   while ~isempty(newNode) & size(c)~=0 & notfound
      [len, path] = c.pop();
      curInd = path(end);
      if curInd == minInd
          curPath = path;
          break;
      end
      explored = [explored, curInd];
      edges = edgeMat(:, curInd);
      indsNo = find(edges ~=0)';
      for i = indsNo
          if ~any(i == explored)
               pathn = [path, i];
              lenn = -1*edges(i) +len;
              dist = sum((endNode - thetaTree(:,i)).^2);
              if dist<curDist
                  curDist = dist;
                  curPath = pathn;
              end
              c.insert(lenn, pathn);
          end
      end
   end
   figure;

   [rr, ~] = size(circles);
   endNodexy = getEndPosition(endNode, armLengths);
   startNodexy = getEndPosition(startNode, armLengths);
   thetaTree = [thetaTree, endNode];
   [rtr, ctr] = size(thetaTree)
   for p  =[ curPath, ctr]
       current_thetas = thetaTree(:, p);
       %finding the coordinates of joint1 
        joint1_x = armLengths(1) * cos(current_thetas(1));
        joint1_y = armLengths(1) * sin(current_thetas(1));
    
        %finding the coordinates of joint2
        joint2_x = joint1_x + armLengths(2) * cos(current_thetas(1) + current_thetas(2));
        joint2_y = joint1_y + armLengths(2) * sin(current_thetas(1) + current_thetas(2));
        
        %finding the location of endPoint
        endPosition_x = joint2_x + armLengths(3) * cos(current_thetas(1) + current_thetas(2) + current_thetas(3));
        endPosition_y = joint2_y + armLengths(3) * sin(current_thetas(1) + current_thetas(2) + current_thetas(3));
         plot(startNodexy(1), startNodexy(2), 'g.', 'MarkerSize', 20);
         hold on;
         plot(endNodexy(1), endNodexy(2), 'm.', 'MarkerSize', 20);
         
         plot([0.0 ; joint1_x], [0.0 ; joint1_y], 'o-'); hold on;
		 plot([joint1_x ; joint2_x], [joint1_y ; joint2_y], 'ro-'); hold on;
         plot([joint2_x ; endPosition_x], [joint2_y ; endPosition_y], 'bo-'); 
         for rrr = 1:2:rr
            plot(circles(rrr,:), circles(rrr+1,:), 'g-');
         end
         axis([-5,5,-5,5]);
         axis square
         axis equal 
         xlabel 'x-distance';
         ylabel 'y-distance';
         %writeVideo(obj2, getframe(gcf));
         hold off;
   end
 
   t-cputime
   numIter+1
   length(curPath)
   
    moves = edgeMat;
    close(obj);
    close(obj2);
end

function isIntersect = getColisions(startNode, endTh, obstacles)
    endCoords = getEndPositions(endTh, [2,2,1]);
    endCoords2 = getEndPosition(startNode, [2,2,1])';
    endCoords = [[0;0], endCoords, endCoords2(1:2)];
    
    isIntersect = false;
    [row, ~] = size(obstacles);
    for r = 1:2:row
        obs = obstacles(r:r+1, :);
        for np = 1:4
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