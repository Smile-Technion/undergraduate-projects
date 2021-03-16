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

plot_Robot(t, l1, l2, theta1, theta2, 3, 3, 0)
