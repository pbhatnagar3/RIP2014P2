% Erion Plaku
% Introduction to Robotics
% Department of Electrical Engineering and Computer Science
% Catholic University of America
%
% http://faculty.cua.edu/plaku/
%
% Copyright 2012 by Erion Plaku
% The content may be reproduced under the obligation to acknowledge the source

function [near] = IsPointNearLine(p, line1, line2)
    % Determine if point p is near the line defined by its two endpoints line1, line2
    x  = line2(1) - line1(1);
    y  = line2(2) - line1(2);
    d  = (p(1) - line1(1)) * y - (p(2) - line1(2)) * x;
    near = d * d / (x * x + y * y) < 0.064;
end
