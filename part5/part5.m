function part5()

addpath('../');

    %ArmLength for the Arm. Start and Goal Configurations. 
    armLength = [2, 2, 1];
    
    %Start and Goal Configuration Given
    %The Matrix is in this format for convenience
    %1.5707
    %-1.2308
    %0
    startConfiguration = [1.5707; -1.2308; 0];
    goalConfiguration  = [1.5707; 1.2308; 0];
    
    %Keep Track of the Number of Edges in the Tree
    numOfEdges = 0;
    %Tree Vertices where the Tree needs to develop
    treeVertices = [startConfiguration];
    
    %ArmE keeps the number of indices in the Arm Matrix.
    armE = [1];

    %For a=1 -> A in the Algorithm A = 1000, Therefore 1000 iterations.
    while(size(treeVertices,2) < 1000)
        
        %Finding the Random value through -pi to pi Q_Rand is needed to be
        %found.
        first = -3.14 + (3.14 - (-3.14)) * rand();
        second = -3.14 + (3.14 - (-3.14)) * rand();
        third = -3.14 + (3.14 - (-3.14)) * rand();
        qRand = [first, second, third]';

        %Find the nearest Value. Q_near with Q_Rand and T. Where T is the
        %matrix treeVertices'
        nearestValue= 1000000;
        Vprime = treeVertices';
        for i=1 : size(Vprime,1)
          value = Vprime(i,:);
          dist= (value(1)-qRand(1))*(value(1)-qRand(1))+(value(2)-qRand(2))*(value(2)-qRand(2))+(value(3)-qRand(3))*(value(3)-qRand(3));
          if(dist<nearestValue)
            index=i;
            nearestValue=dist;
          end
        end
        near =  index;
        %qNear finds the nearest value by using the function to get the
        %index which can be accessed by tree Vertices.
        qNear = treeVertices(:, near);
        
        %qDirectional Value is calculated through the
        %QRand-Qnear/Norm(Qrand-Qnear)
        qDir = qNear + (0.05/norm(qRand - qNear)) * [qRand(1) - qNear(1), qRand(2) - qNear(2), qRand(3) - qNear(3)]' ;
       
        %Calculate the task error.
        %Computing the Task Error by using the Forward Kinematics T^e_0
        forwardKinematicsValue = [];
        forwardKinematicsValue(1) = cos(qDir(1)) * armLength(1) + cos(qDir(1) + qDir(2)) * armLength(2) + cos(sum(qDir(:))) * armLength(3);
        forwardKinematicsValue(2) = sin(qDir(1)) * armLength(1) + sin(qDir(1) + qDir(2)) * armLength(2) + sin(sum(qDir(:))) * armLength(3);
        forwardKinematicsValue(3) = sum(qDir(:));
        position = forwardKinematicsValue';
        
        %Task Coordinates.
        f_y = norm(2.8868 - position(2));
        %End of Calculating the task Error.
         
        %Usage of a temporary variable q_Dir.
        q_Dir = qDir;

        %Calculating the number of nodes or vertices in the present values.
        %Also setting the norm if possible
        
        %A checker method to find the values in this position
        xyVertices = 0;
        while(1==1)
          xyVertices = xyVertices+1;
          if(norm(qDir - q_Dir) > norm(q_Dir - qNear))
            qDir = q_Dir;
            break;
          end
          if norm(f_y) < 0.0005 || xyVertices > 1000
            break
          end
          
          %Calculating the Task Error. 
          forwardKinematicsValue2 = [];
          forwardKinematicsValue2(1) = cos(qDir(1)) * armLength(1) + cos(qDir(1) + qDir(2)) * armLength(2) + cos(sum(qDir(:))) * armLength(3);
          forwardKinematicsValue2(2) = sin(qDir(1)) * armLength(1) + sin(qDir(1) + qDir(2)) * armLength(2) + sin(sum(qDir(:))) * armLength(3);
          forwardKinematicsValue2(3) = sum(qDir(:));
          position = forwardKinematicsValue2';
          f_x = 0;
          f_y = norm(2.8868 - position(2));
          %End of the Calculation. These values will be further used to
          %help provide a normalized vector.
          
          %Theta Values and velocities of the Jacobian vectors are set.
          %These values are needed for the Arm Velocity and Arm moving 
          thetaValue = 0;
          xVelocity = f_x;
          yVelocity = f_y;
          thetaVelocity = thetaValue;
          goalVelocity = [xVelocity; yVelocity; thetaVelocity];
          goalVelocity = 0.05 * goalVelocity;
          Jacboian = getJacobian(qDir, armLength);
          jVelocity = inv(Jacboian) * goalVelocity;
          qDir = qDir - 0.05* jVelocity;    

        end
        %if *CONSTRAINED* NEW CONFIG(qs, qnear) Add the vertices to the
        %matrices.
        if(f_y < 2* 0.2)
            treeVertices = [treeVertices, qDir];
            armE(length(armE) + 1) = near;
            numOfEdges = numOfEdges + 1;
            Edges(numOfEdges, :) = [qNear', qDir'];
        end
        
        %Finding the Task Errors for the QDir to the Goal Configuration.
        forwardKinematicsValue3 = [];
        forwardKinematicsValue3(1) = cos(qDir(1)) * armLength(1) + cos(qDir(1) + qDir(2)) * armLength(2) + cos(sum(qDir(:))) * armLength(3);
        forwardKinematicsValue3(2) = sin(qDir(1)) * armLength(1) + sin(qDir(1) + qDir(2)) * armLength(2) + sin(sum(qDir(:))) * armLength(3);
        forwardKinematicsValue3(3) = sum(qDir(:));
        forwardKinematics_new = forwardKinematicsValue3;
        forwardKinematicsValue4 = [];
        forwardKinematicsValue4(1) = cos(goalConfiguration(1)) * armLength(1) + cos(goalConfiguration(1) + goalConfiguration(2)) * armLength(2) + cos(sum(goalConfiguration(:))) * armLength(3);
        forwardKinematicsValue4(2) = sin(goalConfiguration(1)) * armLength(1) + sin(goalConfiguration(1) + goalConfiguration(2)) * armLength(2) + sin(sum(goalConfiguration(:))) * armLength(3);
        forwardKinematicsValue4(3) = sum(goalConfiguration(:));
        forwardKinematics_goal = forwardKinematicsValue4;
        
        %If Normalized, then break from the while loop.
        if(norm(forwardKinematics_new - forwardKinematics_goal) < 0.2)
            break;
        end

    end

    %Since the values of the tree Vertices are populated, Final Node or the
    %Final vertex is retrieved for the usage of Line Segment (arms).
    finalNode = treeVertices(1:3, end); 
    lineSegment = finalNode;
    xyVertices = length(treeVertices);

    %If the value of the Vertices is greater than zero populate Line
    %Segments. Also make sure the armE is populated. 
    while(xyVertices > 1)
        lineSegment = [lineSegment treeVertices(1:3, xyVertices)];
        xyVertices = armE(xyVertices);  
    end
    %Instantiate tree to all zeros so that it can be populated with Tree
    %vertices. 
    tree = zeros(length(armE), 6);
    
    %Populate the tree variable with the vertices. 
    for i = 1:1:length(armE)
        tree(i, :) = [treeVertices(1:3, i)', treeVertices(1:3, armE(i))'];
    end
    
    %Plot the Graph for the Tree. 
    
    %Plot the start and Goal Configuration. 
    scatter(startConfiguration(1), startConfiguration(2), 50, 'filled'); hold on;
    scatter(goalConfiguration(1), goalConfiguration(2), 100, 'filled'); hold on;
    
    %Start with the values and start drawing the 3d Graph using Plot3
    %method. 
    set(gcf, 'Position', get(0,'Screensize')); 
    count = 1;
    for ii = 1 : size(tree,1)
      if (ii==0)
          break;
      end;
      %Draw the 3D-Point
      if(ii > 1)
          delete(curr);
          plot3(tree(ii-1,[1,4]), tree(ii-1,[2,5]), tree(ii-1,[3,6]), 'k'); hold on;
      end
      curr = plot3(tree(ii,[1,4]), tree(ii,[2,5]), tree(ii,[3,6]), 'o-'); hold on;
      %Start the frames and print the frames for the movie. 
      if(mod(ii,10) == 0)
        name = sprintf('tree%05d',count);
        view(count*3, 28);
        axis([-4 4 -4 4 -4 4]);
        axis square;    
        print(figure(1),name,'-dpng');
        count = count+1;
      end
    end
    
    %After the Tree Graph, Use this function to draw the Arms. 
    drawArms(lineSegment,armLength);   
end

%Function to draw Arms. 
function [] = drawArms(line,q1)
    startConfiguration = [1.5707; -1.2308; 0];
    goalConfiguration  = [1.5707; 1.2308; 0];
    lineValue=line';
    secondCounter = 1;
    armLength = q1;
    for index = size(lineValue, 1) : -1 : 1
        %finding the correct theta values to use
        current_thetas = lineValue(index, :);
        %finding the coordinates of joint1 
        joint1_x = armLength(1) * cos(current_thetas(1));
        joint1_y = armLength(1) * sin(current_thetas(1));
    
        %finding the coordinates of joint2
        joint2_x = joint1_x + armLength(2) * cos(current_thetas(1) + current_thetas(2));
        joint2_y = joint1_y + armLength(2) * sin(current_thetas(1) + current_thetas(2));
        
        %finding the location of endPoint
        endPosition_x = joint2_x + armLength(3) * cos(current_thetas(1) + current_thetas(2) + current_thetas(3));
        endPosition_y = joint2_y + armLength(3) * sin(current_thetas(1) + current_thetas(2) + current_thetas(3));
   
        plot([0.0 ; joint1_x], [0.0 ; joint1_y], 'o-'); hold on;
		plot([joint1_x ; joint2_x], [joint1_y ; joint2_y], 'ro-'); hold on;
        plot([joint2_x ; endPosition_x], [joint2_y ; endPosition_y], 'bo-'); hold on;
        scatter(startConfiguration(1), startConfiguration(2), 50, 'filled'); hold on;
        scatter(goalConfiguration(1), goalConfiguration(2), 100, 'filled'); hold on;
        axis([-10, 10, -10, 10]); hold on;
        axis equal;
        
        name = sprintf('arm%04d',secondCounter);
        print(gcf,name,'-dpng');
        secondCounter = secondCounter + 1;
    end
    set(gcf, 'Position', get(0,'Screensize')); 

    
end
