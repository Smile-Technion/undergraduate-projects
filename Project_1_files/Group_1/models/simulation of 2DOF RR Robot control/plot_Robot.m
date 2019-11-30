function plot_Robot(X1 ,l1, l2, theta1, theta2, X_limit, Y_limit, video)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% X1 = X1';
%create figure elements
hFig = figure(3);
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

c1 = cos(theta1);
s1 = sin(theta1);
c12 = cos(theta1+theta2);
s12 = sin(theta1+theta2);

l1_line_end = [l1.*c1; l1.*s1];
l2_line_end = [l2.*c12 ; l2.*s12]+l1_line_end;

Grip = zeros(2,length(X1(1,:)));  % initiate grip position log vector
hold on
grid on

if video == 1
    v = VideoWriter('video1.avi');
    open(v)
end


for k=1:length(X1(2,:))
    
    Grip(1,k) = l2_line_end(1,k); % log grip position
    Grip(2,k) = l2_line_end(2,k);
    
    set(hArm1,'XData',[0, l1_line_end(1,k)],'YData',[0 , l1_line_end(2,k)]);
    set(hArm2,'XData',[l1_line_end(1,k), l2_line_end(1,k)],'YData',[l1_line_end(2,k) , l2_line_end(2,k)]);
    set(hMark,'XData',Grip(1,k),'YData',Grip(2,k));
    drawnow
    xlim(hAxes,[-X_limit X_limit]);
    
    if video == 1
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    
    plot(Grip(1,:), Grip(2, :), '*g')
   
end

if video == 1
    close(v)
end

end

