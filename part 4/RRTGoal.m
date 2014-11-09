function moves = RRTGoal(startNode, endNode, obstacles)
    t=cputime;
    obj = VideoWriter('GoalTree');
    obj2 = VideoWriter('GoalArm');
    open(obj);
    open(obj2);
    tree = startNode;
    maxIter = 300;
    edgeMat = zeros(maxIter,maxIter);
    goalFound = false;
    thresh = .05;
     armLengths = [2,2,1];
     figure;
     thetas = [0, 0, startNode(3)];
     xst = startNode(1) - cos(thetas(3));
     yst = startNode(2) - sin(thetas(3));
     angle = acos(-1*(xst.^2 + yst.^2 - 8)/8);
     thetas(1) = atan(yst/xst)-pi/2-angle/2;
     thetas(2) =  angle-pi;
     thetas(3) = mod(thetas(3) - thetas(2) - thetas(1), pi);
     thetas = mod(thetas, 2*pi);
     thetaTree = thetas';
     plot(startNode(1), startNode(2), 'g.', 'MarkerSize', 20);
     hold on;
     plot(endNode(1), endNode(2), 'm.', 'MarkerSize', 20);
     %plot(endNode(1), endNode(2), 'm.', 'MarkerSize', 20);
     axis square
     axis equal
     xlabel 'x-distance';
     ylabel 'y-distance';
     axis([-5,5,-5,5]);
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
   while numIter < maxIter-1 & ~goalFound

        newTh1 = mod(baseTheta(1)-rand()*pi/4 + pi/8, 2*pi);
        newTh2 = mod(baseTheta(2)+pi -rand()*pi/4 + pi/8, 2*pi) - pi; 
        newTh3 = mod(baseTheta(3)+pi -rand()*pi/4+ pi/8, 2*pi) - pi; 
        theta = [newTh1, newTh2, newTh3];
        newNode = getEndPosition(theta, armLengths)';
        distanceVec =(thetaTree(1,:) - theta(1)).^2+ (thetaTree(2,:)-theta(2)).^2+(thetaTree(3,:)-theta(3)).^2;
        
       [val, ind] = min(distanceVec);
       nearestTheta = thetaTree(:,ind);
       temp = theta' - nearestTheta;
       thetaVec = temp./(sqrt(sum(temp.^2))*5);
       theta = (nearestTheta + thetaVec)';
       newNode = getEndPosition(theta, armLengths)';
       nearestNode = tree(:,ind);
       count = count +1;
       if ~getColisions(theta, circles)
           numIter = numIter +1;

           distFromGoal = sum((newNode - endNode).^2);
           if distFromGoal < thresh
               goalFound = true;
           end
                tree = [tree, newNode];
                thetaTree = [thetaTree, theta'];
                edgeMat(ind, length(tree)) = val;
                edgeMat(length(tree), ind) = val;
                plot([nearestNode(1), newNode(1)], [nearestNode(2), newNode(2)], 'ko-');        
                writeVideo(obj, getframe(gcf));
       end   
       if count >=10
             distanceVec =(tree(1,:) - endNode(1)).^2+ (tree(2,:)...
                 -endNode(2)).^2+(tree(3,:)-endNode(3)).^2;
           [~, minInd] = min(distanceVec);
           baseTheta = thetaTree(:, minInd);
           count = 0;
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
   distanceVec =(tree(1,:) - endNode(1)).^2+ (tree(2,:)...
                 -endNode(2)).^2+(tree(3,:)-endNode(3)).^2;
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
              dist = sum((endNode - tree(:,i)).^2);
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
   for p  = curPath
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
         plot(startNode(1), startNode(2), 'g.', 'MarkerSize', 20);
         hold on;
         plot(endNode(1), endNode(2), 'm.', 'MarkerSize', 20);
         
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
         writeVideo(obj2, getframe(gcf));
         hold off;
   end
   cputime-t
   numIter
   length(curPath)
   

      
   
    moves = edgeMat;
    close(obj);
    close(obj2);
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