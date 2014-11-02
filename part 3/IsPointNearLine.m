function [near] = IsPointNearLine(p, line1, line2, radius)
% Determine if point p is near the line defined by its two endpoints line1, line2
%     x  = line2(1) - line1(1);
%     y  = line2(2) - line1(2);
%     d  = (p(1) - line1(1)) * y - (p(2) - line1(2)) * x;
%    near = d * d / (x * x + y * y) < (radius * 1.1)
    near = false;
    slope = (line2(2) - line1(2)) / (line2(1) - line1(1));
    if(line1(1) < line2(1))
        x = linspace(line1(1), line2(1), 100);
    else
        x = linspace(line2(1), line1(1), 100);  
    end
    y = slope * x;
    
    if(norm(p - line2) < radius * 1.1)
        near = true;
    end
    
    if(norm(p - line1) < radius * 1.1)
        near = true;
    end
    
    for i = 1: length(x)
        norm(p - [x(i),y(i)]);
        if(norm(p - [x(i),y(i)]) <= radius * 1.1 )
            near = true;
            break;
        end        
    end

end