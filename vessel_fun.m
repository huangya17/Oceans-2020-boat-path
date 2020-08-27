%%                - Function to determine the manoeuvre points -               %%
%                                                                               %
%  input: the start S (XS, YS) and target T (XT, YT) points of each segment,    %
%           the radius of the obstacle located between S and T,                 %
%           the safety radius RB                                                %
%  output: the manoeuvre points (Xa, Ya) and (Xb, Yb) created in order          %
%            to avoid the obstacle                                              %
%                                                                               %
%                Written by Dimitrios Stergianelis on August 2018               %
%                                                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function [Xa, Ya, Xb, Yb, err] = vessel_fun(XS, YS, XT, YT, XO, YO, RO, RB, direction)
 
% Create the manoeuvre points arrays
Xa = [];
Ya = [];
Xb = [];
Yb = [];
 
err = false;
 
% Find the straight-line equation (Y = a*X+b) connecting the start and
% target points
a = (YS - YT)/(XS - XT);
b = (XS*YT - XT*YS)/(XS - XT);
 
% Find the determinant radius RD
RD = RO + RB;
 
% Check if path inside obstacle region
if (sqrt((XS - XO)^2 + (YS - YO)^2) < RD) || (sqrt((XT - XO)^2 + (YT - YO)^2) < RD)
    err = true;
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
check_3 = (D <= 0); % 0 or 1 solutions, i.e. none or one intersection point
 
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
    
    %% Way of updating the position vector
    if ~(NoIntersection)
        % Determine the direction to turn
        
        % Plot the intersection points
        plot(X1, Y1, 'xk')
        plot(X2, Y2, 'xk')
        
        if (XS == XT) % Extra criterion because in this case YCRIT=YO and cannot determine the direction to turn
            if (XS >= XO)
                if (YS > YO) % First quadrant
                    CCW = true;
                else % YS < YO Fourth quadrant
                    CCW = false;
                end
            else % XS < XO
                if (YS > YO) % Second quadrant
                    CCW = false;
                else % YS < YO Third quadrant
                    CCW = true;
                end
            end
            XC = XS;
            YC = YO;
            
        else % XS not equal with XT
            if (YS == YT)
                YCRIT = YS;
                XC = XO;
                YC = YS;
            else % Calculate the YCRIT
                YCRIT = a*XO + b;
                % Find the line equation (Y = a2*X + b2) which is lateral to the
                % initial one and is crossing from the centre of the obstacle
                a2 = -1/a;
                b2 = (a*YO + XO)/a;
                % Find the cross point of the two lines (XC, YC)
                % Solve the system (Y = a*X + b) and (Y = a2*X + b2)
                XC = (b2 - b)/(a - a2);
                YC = a*XC + b;
            end
            
            % If YCRIT>=YO then turn CCW angle f1 and CW f2
            if (YCRIT >= YO)
                CCW = true;
            else % If YCRIT<YO then turn CW angle f1 and CCW f2
                CCW = false;
            end
        end
        
        % Calculate the distance from the centre of the obstacle to the cross point
        LR = sqrt((XC - XO)^2 + (YC - YO)^2);
        
        % Calculate the distance from the cross point to the circumference
        if direction
            LM = RD - LR;
        else
            LM = RD + LR;
        end
        
        % Calculate the distances from start point to cross point
        tmp0 = sqrt((X1 - XS)^2 + (Y1 - YS)^2);
        tmp1 = sqrt((X2 - XS)^2 + (Y2 - YS)^2);
        
        % SET the smaller D1 and the bigger D2
        if (tmp0 < tmp1)
            D1 = tmp0;
        else
            D1 = tmp1;
        end
        
        % Calculate the distance between the cross points
        LP = sqrt((X1 - X2)^2 + (Y1 - Y2)^2);
        
        % Calculate the length of hypotenuse in the start triangle
        L1 = sqrt(LM^2 + D1^2);
        
        % Calculate the turn angle
        % If YCRIT>=YO then turn CW/CCW angle f1
        if (CCW == true)
            if direction
                f = atan(LM/D1);
                f1 = f + 2*pi/3600; % Extra angle added to compensate for numerical inaccuracy
            else
                f = -atan(LM/D1);
                f1 = f - 2*pi/3600; % Extra angle added to compensate for numerical inaccuracy
            end
        else % If YCRIT<YO then turn CW angle f1
            if direction
                f = -atan(LM/D1);
                f1 = f - 2*pi/3600; % Extra angle added to compensate for numerical inaccuracy
            else
                f = atan(LM/D1);
                f1 = f + 2*pi/3600; % Extra angle added to compensate for numerical inaccuracy
            end
        end
        % The extra angle is equal to 0.1 degrees and is going to play an
        % insignificant role to the trajectory length while will solve the
        % rounding decimals problem
        
        %% Finding the manoeuvre points
        if (XT > XS) % Forward motion
            % First manoeuvre point
            Xa = XS + L1*cos(atan(a) + f1);
            Ya = YS + L1*sin(atan(a) + f1);
            
            % Second manoeuvre point
            Xb = Xa + LP*cos(atan(a));
            Yb = Ya + LP*sin(atan(a));
            
        elseif (XS == XT) % Parallel to Y-axis motion
            % First manoeuvre point
            Xa = XS + sign(YS-YO)*L1*cos(pi/2 - f1);
            Ya = YS - sign(YS-YO)*L1*sin(pi/2 - f1);
            
            % Second manoeuvre point
            Xb = Xa - sign(YS - YO)*LP*cos(pi/2);
            Yb = Ya - sign(YS - YO)*LP*sin(pi/2);
            
        else % Backward motion
            % First manoeuvre point
            Xa = XS - L1*cos(atan(a) - f1);
            Ya = YS - L1*sin(atan(a) - f1);
            
            % Second manoeuvre point
            Xb = Xa - LP*cos(atan(a));
            Yb = Ya - LP*sin(atan(a));
        end
    end
end
end

