% Erion Plaku
% Introduction to Robotics
% Department of Electrical Engineering and Computer Science
% Catholic University of America
%
% http://faculty.cua.edu/plaku/
%
% Copyright 2012 by Erion Plaku
% The content may be reproduced under the obligation to acknowledge the source
%
% Modified 2014 by Kenneth Marino
%
function [near] = ArePointsNear(pt1, pt2, eps)
    near = norm(pt1 - pt2) < eps;
end