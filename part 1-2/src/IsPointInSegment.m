% Kenneth Marino 2014
%
function [in] = IsPointInSegment(p, line1, line2)
    % Determine if point p is in the correct range to be on line defined by its two endpoints line1, line2
    if ((p(1) < line1(1)) && (p(1) < line2(1))) || ... 
            ((p(1) > line1(1)) && (p(1) > line2(1))) || ...
            ((p(2) < line1(2)) && (p(2) < line2(2))) || ... 
            ((p(2) > line1(2)) && (p(2) > line2(2)))
        in = 0;    
    else
        in = 1;
    end
end