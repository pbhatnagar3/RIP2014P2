% to figure out the Jacobian 
% Input : ang : the different angles
% Output: Jacobian : the Jacobian for the equation
function Jacobian = getJacobian(ang,len)

    dxq1 = - len(1)*sin(ang(1)) - len(2)*sin(ang(1)+ang(2)) - len(3)*sin(ang(1)+ang(2)+ang(3));
    dxq2 = - len(2)*sin(ang(1)+ang(2)) - len(3)*sin(ang(1)+ang(2)+ang(3));
    dxq3 = - len(3)*sin(ang(1)+ang(2)+ang(3));
    dyq1 = len(1)*cos(ang(1)) + len(2)*cos(ang(1)+ang(2)) + len(3)*sin(ang(1)+ang(2)+ang(3));
    dyq2 = len(2)*cos(ang(1)+ang(2)) + len(3)*cos(ang(1)+ang(2)+ang(3));
    dyq3 = len(3)*cos(ang(1)+ang(2)+ang(3));


    Jacobian = [dxq1 dxq2 dxq3; dyq1 dyq2 dyq3; 1 1 1];


end