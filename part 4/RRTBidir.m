function moves = RRTBidir(startNode, endNode, obstacles)
    obj = VideoWriter('BiDirTree');
    obj2 = VideoWriter('BiDirArm');
    open(obj);
    open(obj2);
    tree = startNode;
    treeG = endNode;
    maxIter = 300;
    edgeMat = zeros(maxIter,maxIter);
    edgeMatG = zeros(maxIter,maxIter);
    goalFound = false;
    thresh = 2;
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
     thetasEnd = [1.9199; 2.0164; 0.2848];
     thetaTreeG = thetasEnd;
     gNode1 =endNode;
     gNode2 = startNode;
     
     
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
   baseThetaG = thetaTreeG(:,1);
   count = 0;
   numIter = 0;
   while numIter < maxIter-1 & ~goalFound
        nio = numIter;
        newTh1 = mod(baseTheta(1)-rand()*pi/4 + pi/8, 2*pi);
        newTh2 = mod(baseTheta(2)+pi -rand()*pi/4 + pi/8, 2*pi) - pi; 
        newTh3 = mod(baseTheta(3)+pi -rand()*pi/4+ pi/8, 2*pi) - pi; 
        
        newTh1G = mod(baseThetaG(1)-rand()*pi/4 + pi/8, 2*pi);
        newTh2G= mod(baseThetaG(2)+pi -rand()*pi/4 + pi/8, 2*pi) - pi; 
        newTh3G = mod(baseThetaG(3)+pi -rand()*pi/4+ pi/8, 2*pi) - pi; 
        
        theta = [newTh1, newTh2, newTh3];
        thetaG = [newTh1G, newTh2G, newTh3G];

        
        distanceVec =(thetaTree(1,:) - theta(1)).^2+ (thetaTree(2,:)-theta(2)).^2+(thetaTree(3,:)-theta(3)).^2;
        
       [val, ind] = min(distanceVec);
       newNode = getEndPosition(theta, armLengths)';
       nearestNode = tree(:,ind);
       count = count +1;
       if ~getColisions(nearestNode, theta, circles)
                numIter = numIter +1;
                tree = [tree, newNode];
                thetaTree = [thetaTree, theta'];
                edgeMat(ind, length(tree)) = val;
                edgeMat(length(tree), ind) = val;
                plot([nearestNode(1), newNode(1)], [nearestNode(2), newNode(2)], 'ko-'); 
                distanceVec =(thetaTreeG(1,:) - theta(1)).^2+ (thetaTreeG(2,:)...
            -theta(2)).^2+(thetaTreeG(3,:)-theta(3)).^2;              
            [val, ind] = min(distanceVec);
            if val < thresh
                if ~getColisions(treeG(:, ind), theta, circles)
                    nn = treeG(:,ind);
                   goalFound = true;
                   connectNode1 = length(tree);
                   connectNode2 = ind;
                   distFin = val;
                    plot([nn(1), newNode(1)], [nn(2), newNode(2)], 'ko-');
                end
            end  
       end   
       
       
        distanceVec =(thetaTreeG(1,:) - thetaG(1)).^2+ (thetaTreeG(2,:)...
            -thetaG(2)).^2+(thetaTreeG(3,:)-thetaG(3)).^2;
        
       [val, ind] = min(distanceVec);
       newNodeG = getEndPosition(thetaG, armLengths)';
       nearestNode = treeG(:,ind);
  
       if ~getColisions(nearestNode, thetaG, circles) &&~goalFound
           numIter = numIter +1;
                treeG = [treeG, newNodeG];
                thetaTreeG = [thetaTreeG, thetaG'];
                edgeMatG(ind, length(treeG)) = val;
                edgeMatG(length(treeG), ind) = val;
                plot([nearestNode(1), newNodeG(1)], [nearestNode(2), newNodeG(2)], 'ko-');
                 distanceVec =(thetaTree(1,:) - thetaG(1)).^2+ (thetaTree(2,:)...
            -thetaG(2)).^2+(thetaTree(3,:)-thetaG(3)).^2;              
            [val, ind] = min(distanceVec);
             if val < thresh & ~goalFound
                 if ~getColisions(tree(:, ind), thetaG, circles)
                     nn = tree(:,ind);
                   goalFound = true;
                   connectNode1 = ind;
                   connectNode2 = length(treeG);
                   distFin = val;
                   plot([nn(1), newNodeG(1)], [nn(2), newNodeG(2)], 'ko-');
                 end
            end 
               
       end   
       if numIter ~= nio
           writeVideo(obj, getframe(gcf));
       end
       
       if count >=10
            newTh1 = rand()*2*pi;
            newTh2 = rand()*2*pi-pi;
            newTh3 =rand()*2*pi-pi;
            temp = [newTh1;newTh2;newTh3];
            goal = getEndPosition(temp', armLengths);
            distanceVec =(tree(1,:) - goal(1)).^2+ (tree(2,:)...
                 -goal(2)).^2+(tree(3,:)-goal(3)).^2;
           [~, minInd] = min(distanceVec);
           baseTheta = thetaTree(:, minInd);
           
           distanceVec =(treeG(1,:) - goal(1)).^2+ (treeG(2,:)...
                 -goal(2)).^2+(treeG(3,:)-goal(3)).^2;
           [~, minInd] = min(distanceVec);
           baseThetaG = thetaTreeG(:, minInd);
           count = 0;
       end
   end
   buffer = zeros(maxIter,maxIter);
   edgeMat = [edgeMat, buffer; buffer, edgeMatG];
   if goalFound
       edgeMat(connectNode1, connectNode2+maxIter) = distFin;
        edgeMat(connectNode2+maxIter, connectNode1) = distFin;
   end
   [rrrr, cccc] = size(treeG);

       tree(:, maxIter+1:maxIter+cccc)=treeG;
       thetaTree(:, maxIter+1:maxIter+cccc) = thetaTreeG;
 
   
   
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

   
    moves = edgeMat;
    close(obj);
    close(obj2);
end
function isIntersect = getColisions(startNode, endTh, obstacles)
    endCoords = getEndPositions(endTh, [2,2,1]);
    endCoords = [[0;0], endCoords, [startNode(1);startNode(2)]];
    
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