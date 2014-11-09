% write function that projects the  point (q = X,Y) on a vector
% which is composed of two points - vector = [p0x p0y; p1x p1y]. 
% i.e. vector is the line between point p0 and p1. 
%
% The result is a point qp = [x y] and the length [length_q] of the vector drawn 
% between the point q and qp . This resulting vector between q and qp 
% will be orthogonal to the original vector between p0 and p1. 
% 
% This uses the maths found in the webpage:
% http://cs.nyu.edu/~yap/classes/visual/03s/hw/h2/math.pdf
%
function [ProjPoint, length_q] = ProjectPoint(vector, q)
      p0 = vector(1,:);
      p1 = vector(2,:);
      length_q = 1; %ignore for now
      a = [p1(1) - p0(1), p1(2) - p0(2); p0(2) - p1(2), p1(1) - p0(1)];
      b = [q(1)*(p1(1) - p0(1)) + q(2)*(p1(2) - p0(2)); ...
          p0(2)*(p1(1) - p0(1)) - p0(1)*(p1(2) - p0(2))] ;
      ProjPoint = a\b;
      ProjPoint = ProjPoint';
end