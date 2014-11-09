function jointPos = getJointPosition(l, ang)
jointPos = [0,0;0,0;0,0];
for i = 1:3
jointPos(i, 1) = l(i)*cos(ang(i));
jointPos(i, 2) = l(i)*sin(ang(i));
end
for i = 2:3
jointPos(i, 1) = jointPos(i - 1, 1) + jointPos(i, 1);
jointPos(i, 2) = jointPos(i - 1, 2) + jointPos(i, 2);
end
% j1_x = l(1)*cos(ang(1));
% j1_y = l(1)*sin(ang(1));
%
% j2_x = j1_x + l(2)*cos(ang(2));
% j2_y = j2_y + l(2)*sin(ang(2));
%
% j3_x = l(1)*cos(ang(1));
% j3_y = l(1)*sin(ang(1));
end