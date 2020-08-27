%%                        - Function to plot obstacles -                       %%
%          This code introduces a function to plot the obstacle array           %
%                                                                               %
%                Written by Dimitrios Stergianelis on August 2018               %
%                                                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function [RD] = plot_obstacles(XS, YS, XT, YT, XO, YO, RO, RB, N)
%% Plotting basic features
 
% Add description to data points S and T
txt1 = ' Start point';
text(XS,YS,txt1,'VerticalAlignment','bottom')
hold on; box on; xlabel('X (m)'); ylabel('Y (m)');
 
txt2 = ' Target point';
text(XT,YT,txt2,'VerticalAlignment','bottom')
 
% Plot straight line from S to T with black dotted line
plot([XS XT], [YS YT], ':xk')
 
% plot(XS, YS, ':xk')
% plot(XT, YT, ':xk')
 
% Plot all obstacles
RD = zeros(1, N);
for i = 1:N
    % Find the determinant radius RD for the obstacle region
    RD(i) = RO(i) + RB;
    
    % Plot centre of obstacle circle
    plot(XO(i), YO(i), '.b')
    
    % Plot circle (X-XO)^2+(Y-YO)^2=RD^2 with XO, YO and RD red --- line
    plot_circle(XO(i), YO(i), RD(i), 'r');
    
    % Plot circle (X-XO)^2+(Y-YO)^2=RO^2 with XO, YO and RO blue --- line
    plot_circle(XO(i), YO(i), RO(i), 'b');
    
    if (i == 1)
        axis equal
    end
end
end