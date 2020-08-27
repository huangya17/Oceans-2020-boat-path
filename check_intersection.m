%%                     - Function for path simplification -                    %%
%          This code introduces a function to check if the line segment         %
%              connecting the i and i+2 manoeuvre points intersects             %
%                      with any of the given obstacles                          %
%                                                                               %
%              If no intersection exists the i+1 point is skipped               %
%                                                                               %
%                Written by Dimitrios Stergianelis on August 2018               %
%                                                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function [NoIntersection, err] = check_intersection(XS, YS, XT, YT, XO, YO, RO, RB)
 
err = false;
NoIntersection = true;
 
% Find the straight-line equation (Y = a*X+b) connecting the start and
% target points
a = (YS - YT)/(XS - XT);
b = (XS*YT - XT*YS)/(XS - XT);
 
% Find the determinant radius RD
RD = RO + RB;
 
% Check if path inside obstacle region
if (sqrt((XS - XO)^2 + (YS - YO)^2) < RD) || (sqrt((XT - XO)^2 + (YT - YO)^2) < RD)
    err = true;
    NoIntersection = false;
    return
end
 
% Find the interception point(s) between the line and the circle (equation: (X-XO)^2+(Y-YO)^2 = RD^2)
% Need to solve: (a^2 + 1)*X^2 + 2*(a*b - a*YO - XO)*X + (YO^2 - RD^2 + XO^2 - 2*b*YO + b^2) = 0
% Substitutions in two quadratic equation coefficients
A = (a^2 + 1);
B = 2*(a*b - a*YO - XO);
C = (YO^2 - RD^2 + XO^2 - 2*b*YO + b^2);
 
% Determinant calculation
D = B^2 - 4*A*C;
 
%% Finding the relative position between the straight line and the obstacle
check_1 = ((XS == XT) && ((XS <= XO - RD) || (XS >= XO + RD))); % Route parallel to Y-axis and no intersection points
check_2 = ((YS == YT) && ((YS <= YO - RD) || (YS >= YO + RD))); % Route parallel to X-axis and no intersection points
check_3 = (D < 0); % 0 or 1 solution, i.e. none or one intersection point
 
% Two solutions (intersection points), with coordinates (X1, Y1) & (X2, Y2)
if ~(check_1 || check_2 || check_3)
    
    if (XS == XT) % Route parallel to Y-axis
        X1 = XS;
        X2 = XS;
        Y1 = YO - sqrt (RD^2 - (XS-XO)^2);
        Y2 = YO + sqrt (RD^2 - (XS-XO)^2);
        
    elseif (YS == YT) % Route parallel to X-axis
        X1 = XO - sqrt (RD^2 - (YS-YO)^2);
        X2 = XO + sqrt (RD^2 - (YS-YO)^2);
        Y1 = YS;
        Y2 = YS;
        
    else % Route with random orientation
        X1 = (-B + sqrt(B^2 - 4*A*C))/(2*A);
        X2 = (-B - sqrt(B^2 - 4*A*C))/(2*A);
        Y1 = a*X1 + b;
        Y2 = a*X2 + b;
    end
    
    % Check if the intersection points belong to the line segment from S to T
    NoIntersection = true;
    if (XT > XS)
        if (XS < X1 && X2 < XT)
            NoIntersection = false;
        end
    elseif (XT == XS)
        if (YT > YS)
            if (YS < Y1 && Y2 < YT)
                NoIntersection = false;
            end
        else % YT < YS
            if (YT < Y1 && Y2 < YS)
                NoIntersection = false;
            end
        end
    else % XT < XS
        if (XT < X1 && X2 < XS)
            NoIntersection = false;
        end
    end
end
end

