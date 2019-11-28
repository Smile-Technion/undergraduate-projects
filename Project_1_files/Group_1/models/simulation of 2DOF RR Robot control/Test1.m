close all; clear variables; clc
l1 = 5;
l2 = 5;
X = [1 1];
Xf = [3, 3];
t = linspace(0, 10, 100);

[X,Y, X_dot, Y_dot, X_2dot, Y_2dot] = Motion_plan(X, Xf, t);

q = inv_kin(X,Y,l1,l2);

plot_Robot(t,l1,l2,q(1,:),q(2,:), 5, 5, 0)

% figure()
% plot(t,X)
% 
% figure()
% plot(t,Y_dot)
% 
% figure()
% plot(t,Y_2dot)
%%
x = 0:pi/4:2*pi; 
v = sin(x);
% Define the query points to be a finer sampling over the range of x.

xq = 0:pi/32:2*pi;
% Interpolate the function at the query points and plot the result.

figure
vq1 = interp1(x,v,xq);
plot(x,v,'o',xq,vq1,':.');
xlim([0 2*pi]);
title('(Default) Linear Interpolation');