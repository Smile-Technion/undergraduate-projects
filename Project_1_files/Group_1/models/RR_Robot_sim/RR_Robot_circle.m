clear all; close all; clc

l1 = 2;
l2 = 2;
dt = 0.05;     % time diff
t = dt:dt:6.5;   % time vector

% circle
px = 1.5+cos(t);    % required x position 
py = -1.5+sin(t);    % required y position
% plot(px,py)     % check path

% % arm extention
% px = linspace(1/sqrt(2), 3/sqrt(2), length(t));    % required x position 
% py = linspace(1/sqrt(2), 3/sqrt(2), length(t));    % required y position

%inverse kinematics
c2 = (px.^2 + py.^2 - l1^2 - l2^2)./(2*l1*l2) ; 
s2 = sqrt(1-c2.^2);
theta2 = atan2(s2, c2);

k1 = l1+l2*c2;
k2 = l2*s2;

c1 = (k1.*px + k2.*py)./ (k1.^2+k2.^2) ;   
s1 = (-k2.*px + k1.*py)./ (k1.^2+k2.^2) ;
theta1 = atan2(s1, c1);

%% plot 
hFig = figure(1);
hAxes = axes(hFig);
hold on
xlim(hAxes,[-3 3]);
ylim(hAxes,[-3 3]);
grid on
axis equal
hMark = plot(nan,nan,'ro','MarkerSize',7);
hArm1 = line([nan,nan],[nan,nan],'LineWidth',3);
set(hArm1, 'color', 'k')
hArm2 = line([nan,nan],[nan,nan],'LineWidth',3);

l1_line_end = [l1.*c1; l1.*s1];
l2_line_end = [l2.*cos(theta1+theta2) ; l2.*sin(theta1+theta2)]+l1_line_end;

Grip = zeros(2,length(t));  % initiate grip position log vector
hold on
grid on

v = VideoWriter('video1.avi');
open(v)

for k=1:length(t)
    
    Grip(1,k) = l2_line_end(1,k); % log grip position
    Grip(2,k) = l2_line_end(2,k);
    
    set(hArm1,'XData',[0, l1_line_end(1,k)],'YData',[0 , l1_line_end(2,k)]);
    set(hArm2,'XData',[l1_line_end(1,k), l2_line_end(1,k)],'YData',[l1_line_end(2,k) , l2_line_end(2,k)]);
    set(hMark,'XData',Grip(1,k),'YData',Grip(2,k));
    drawnow
    xlim(hAxes,[-3 3]);
    frame = getframe(gcf);
    writeVideo(v,frame);
    plot(Grip(1,:), Grip(2, :), '*g')
   
end

close(v)