%%        - Function to determine the direction to avoid an obstacle -         %%
%        This code introduces a function to execute the vessel_fun code         %
%                           with two different directions                       %
%        initially finding the short path and if it is not possible to          %
%           execute the vessel_fun code again finding the long path             %
%                                                                               %
%                Written by Dimitrios Stergianelis on August 2018               %
%                                                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function [Xa, Ya, Xb, Yb, err] = vessel_find_path(XS, YS, XT, YT, XO, YO, RO, RB, XO_ARR, YO_ARR, RO_ARR)
%% Core calculations
 
% Try the fast route around the obstacle
[Xa, Ya, Xb, Yb, err] = vessel_fun(XS, YS, XT, YT, XO, YO, RO, RB, true);
 
if ~isempty(Xa)
    for j = 1:length(XO_ARR)
        
        % Start point within obstacle
        check1 = (sqrt((Xa - XO_ARR(j))^2 + (Ya - YO_ARR(j))^2) < (RO_ARR(j) + RB));
        
        % End point within obstacle
        check2 = (sqrt((Xb - XO_ARR(j))^2 + (Yb - YO_ARR(j))^2) < (RO_ARR(j) + RB));
        
        % Take the slow route around the obstacle
        if (check1 || check2)
            [Xa, Ya, Xb, Yb, err] = vessel_fun(XS, YS, XT, YT, XO, YO, RO, RB, false);
        end
    end
end
 
end

