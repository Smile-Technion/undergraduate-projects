function [outputArg1,outputArg2] = plot_Robot(X_limit, Y_limit)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

hFig = figure(1);
hAxes = axes(hFig);
hold on
xlim(hAxes,[-X_limit X_limit]);
ylim(hAxes,[-Y_limit Y_limit]);
grid on
axis equal
hMark = plot(nan,nan,'ro','MarkerSize',7);
hArm1 = line([nan,nan],[nan,nan],'LineWidth',3);
set(hArm1, 'color', 'k')
hArm2 = line([nan,nan],[nan,nan],'LineWidth',3);

l1_line_end = [l1.*c1; l1.*s1];
l2_line_end = [l2.*cos(theta1+theta2) ; l2.*sin(theta1+theta2)]+l1_line_end;

Grip = zeros(2,length(X1(:,1)));  % initiate grip position log vector
hold on
grid on

for k=1:length(X1(:,1))
    
    Grip(1,k) = l2_line_end(1,k); % log grip position
    Grip(2,k) = l2_line_end(2,k);
    
    set(hArm1,'XData',[0, l1_line_end(1,k)],'YData',[0 , l1_line_end(2,k)]);
    set(hArm2,'XData',[l1_line_end(1,k), l2_line_end(1,k)],'YData',[l1_line_end(2,k) , l2_line_end(2,k)]);
    set(hMark,'XData',Grip(1,k),'YData',Grip(2,k));
    drawnow
    xlim(hAxes,[-X_limit X_limit]);
    pause(0.5)
%     plot(Grip(1,:), Grip(2, :), '*g')
   
end

outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

