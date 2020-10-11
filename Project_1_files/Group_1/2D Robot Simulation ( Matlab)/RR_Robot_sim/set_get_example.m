clc;clear all;close all

hFig = figure(1);
hAxes = axes(hFig);
hold on
xlim(hAxes,[-10 10]);
ylim(hAxes,[-10 10]);
grid on
axis equal
hMark = plot(nan,nan,'ro','MarkerSize',7);
hArm = line([nan,nan],[nan,nan],'LineWidth',3);

R = 5;
t = 0:0.1:2*pi;

% UD.SimDone = 0;
% 
% set(hFig,'UserData',UD)

for i = 1:length(t)
set(hArm,'XData',R*[ 0  , cos(t(i))],'YData',R*[0 , sin(t(i))]);
set(hMark,'XData',R*cos(t(i)),'YData',R*sin(t(i)));

xlim(hAxes,[-10 10]);
ylim(hAxes,[-10 10]);

drawnow

end

% UD = get(hFig,'UserData');
% UD.SimDone = 1;
% set(hFig,'UserData',UD)

