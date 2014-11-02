function [near] = IsPointNearLineModified(p, line1, line2, radius)
    near = false;
    if(norm(p-line1) <= radius)
        near = true;
    end
    if(norm(p-line2) <= radius)
        near = true;
    end
    
end