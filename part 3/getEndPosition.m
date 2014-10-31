function endPosition = getEndPosition(thetas, armLength)

%finding the x and y coordinates of joint1 using the fact that the 
%rectangle coordinates can be found using basic sin and cosine laws
joint1_x = armLength(1) * cos(thetas(1));
joint1_y = armLength(1) * sin(thetas(1));

%doing the same for joint2. 
%NOTE: we will have to add the respective coordinates of x and y since the 
%      since the joint2 is on the upper right side of joint1
joint2_x = joint1_x + armLength(2) * cos(thetas(1) + thetas(2));
joint2_y = joint1_y + armLength(2) * sin(thetas(1) + thetas(2));

%finding the location of endPoint
endPosition_x = joint2_x + armLength(3) * cos(thetas(1) + thetas(2) + thetas(3));
endPosition_y = joint2_y + armLength(3) * sin(thetas(1) + thetas(2) + thetas(3));

%now, we can figure out the value of thetas for the end location with
%respect to the origin using inverseTan/ arctan

%we are using the atan2 since it returns the 4 quadrant inverse tangent
%essentially, we are doing the following thetas = tan^-1(rise/ run) 
theta = atan2((endPosition_y - joint2_y), (endPosition_x - joint2_x));
endPosition = [endPosition_x endPosition_y theta];
end
