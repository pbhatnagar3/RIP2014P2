% Kenneth Marino 2014
%
function move = MakeStepOnLine(cur, goal, step)
    % Make a step from cur to goal
    xtogoal = goal(1) - cur(1);
    ytogoal = goal(2) - cur(2);
    disttogoal = sqrt(xtogoal*xtogoal + ytogoal*ytogoal);
    
    % If already within step of goal, just move to goal
    if disttogoal < step
       move(1) = goal(1) - cur(1);
       move(2) = goal(2) - cur(2);
    % Otherwise determine point on that step
    else
        move = [0, 0];
        move(1) = xtogoal * step / disttogoal;
        move(2) = ytogoal * step / disttogoal;
    end
end