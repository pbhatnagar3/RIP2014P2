

function part3()
close all;
%hardcoding the starting position
startPosition = [2.6, 1.3, 1];
%hardcoding the goal position
goalPosition = [-1.4, 1.6, -2];
%thetas
%     thetas = [1.23, -2.01, 1.82];
thetas = [1.23, -2.01, 1.82];
%length
armLengths = [2,2,1];
thetasValueList = [thetas];

%************************************************
%TODO: we still have to tweak the values of threshold and stepValue
threshold = 0.1;
stepValue = 0.1;
%************************************************

%endPoint = startPosition;
%variable to keeptrack of the number of iterations
counter = 0;
%finding the distance from the goal so that we can use it when to
%stop the while loop
distanceFromGoal = norm(goalPosition - startPosition);

%parameters for part b
circleCenter = [-1.5, 3];
radius = 1;
ang=0:0.01:2*pi;
xp=radius*cos(ang);
yp=radius*sin(ang);
thetaThreshold = 0.1;

while (distanceFromGoal > threshold)
    
    
%     if ((norm(circleCenter - endPoint(1:2))) < radius * 1.3)

     joint1Coordinates = [armLengths(1) * cos(thetas(1)), armLengths(1) * sin(thetas(1))];
     joint2Coordinates = [joint1Coordinates(1) + armLengths(2) * cos(thetas(1) + thetas(2)), joint1Coordinates(2) + armLengths(2) * sin(thetas(1) + thetas(2))];
     
            
%      if(joint2Coordinates(2) > 2.4 && joint2Coordinates(2) < 2.6 && joint2Coordinates(1) > -1 && joint2Coordinates(1) < -0.97)
%         sprintf('I am here') 
%      end      
%      if (IsPointNearLineModified(circleCenter, [0, 0], joint1Coordinates, radius))        
%          sprintf('decreasing value of theta 1')
% %          delta = delta * -1;
%         thetas(1) = thetas(1) - thetaThreshold; 
%          joint1Coordinates = [armLengths(1) * cos(thetas(1)), armLengths(1) * sin(thetas(1))];
%     
%     end
%     
%     while (IsPointNearLineModified(circleCenter, joint1Coordinates, joint2Coordinates, radius))        
%         sprintf('decreasing value of theta 2') 
%         thetas(2) = thetas(2) + rand();
%         thetas(1) = thetas(1) + rand();
%         joint1Coordinates = [armLengths(1) * cos(thetas(1)), armLengths(1) * sin(thetas(1))];  
%         joint2Coordinates = [joint1Coordinates(1) + armLengths(2) * cos(thetas(1) + thetas(2)), joint1Coordinates(2) + armLengths(2) * sin(thetas(1) + thetas(2))];
%     end

    
   
    
    count = 1;
    while(norm(circleCenter - joint2Coordinates) <= radius * 1.1 && count <40)
%             thetas(1) = thetas(1) + 0.314;
%             thetas(2) = thetas(2) + .6;
            thetas(1) = thetas(1) + 0.00914;
            joint1Coordinates = [armLengths(1) * cos(thetas(1)), armLengths(1) * sin(thetas(1))];
            joint2Coordinates = [joint1Coordinates(1) + armLengths(2) * cos(thetas(1) + thetas(2)), joint1Coordinates(2) + armLengths(2) * sin(thetas(1) + thetas(2))];
            count = count + 1;
            if(count == 19)
            sprintf('yo');    
            end
    end
    

    %find the location of the end point
    endPoint = getEndPosition(thetas, armLengths);
    
    %find the difference between the goal and the current position
    %we also have to normalize it by dividing it by the distance
    %note: norm gives the equilidean distance
    delta = (goalPosition - endPoint)/norm(goalPosition - endPoint);
    %finding the Jacobian
    J = getJacobian(thetas, armLengths);
    
    %using the equation given in the question, we find the updated
    %theta
    if (counter ~=0)
        thetas = thetas';
    end
    thetas = thetas' + stepValue * inv(J) * delta';
    thetasValueList = [thetasValueList ; thetas'];
    %this time, we will find the distance from the goal as the
    %equidilean distance between the goalPosition and the updated
    %endPoint
    distanceFromGoal = norm(goalPosition - endPoint);
    counter = counter + 1;
end

secondCounter = 1;
for index = 1: 1: size(thetasValueList, 1)
    %finding the correct theta values to use
    current_thetas = thetasValueList(index, :);
    
    %finding the coordinates of joint1
    joint1_x = armLengths(1) * cos(current_thetas(1));
    joint1_y = armLengths(1) * sin(current_thetas(1));
    
    %finding the coordinates of joint2
    joint2_x = joint1_x + armLengths(2) * cos(current_thetas(1) + current_thetas(2));
    joint2_y = joint1_y + armLengths(2) * sin(current_thetas(1) + current_thetas(2));
    
    %finding the location of endPoint
    endPosition_x = joint2_x + armLengths(3) * cos(current_thetas(1) + current_thetas(2) + current_thetas(3));
    endPosition_y = joint2_y + armLengths(3) * sin(current_thetas(1) + current_thetas(2) + current_thetas(3));
    
    plot([0.0 ; joint1_x], [0.0 ; joint1_y], 'o-'); hold on;
    plot([joint1_x ; joint2_x], [joint1_y ; joint2_y], 'ro-'); hold on;
    plot([joint2_x ; endPosition_x], [joint2_y ; endPosition_y], 'bo-'); hold on;
    scatter(startPosition(1), startPosition(2), 50, 'filled'); hold on;
    scatter(goalPosition(1), goalPosition(2), 100, 'filled'); hold on;
    %         plot(circleCenter(1)+xp, circleCenter(2)+yp); hold on;
    
    % Create a purple transparent circle
     x = radius *sin(-pi:0.1*pi:pi) + circleCenter(1);
     y = radius*cos(-pi:0.1*pi:pi) + circleCenter(2);
%     xc = 3.0;
%     yc = 1.0;
%     r = 0.5;
%     x = r*sin(-pi:0.1*pi:pi) + xc;
    c = [0.6 0 1];
    fill(x, y, c, 'FaceAlpha', 0.4)
    axis square
    axis equal
    
    axis([-5, 5, -5, 5]); hold off;
    axis equal
    
    
    name = sprintf('3b-%04d',secondCounter);
    print(gcf,name,'-dpng');
    secondCounter = secondCounter + 1;
end


end
