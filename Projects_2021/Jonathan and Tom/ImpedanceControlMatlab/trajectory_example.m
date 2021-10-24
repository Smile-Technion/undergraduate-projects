% p.traj contains vertices of the trajectory.
% format = [x1 y1; x2 y2; ...]
% It will automatically turn into a minimum jerk trajectory over time
% via function. Below are some examples.

% octagon 
p.traj = [-1.5, 0;-1, 1; 0, 1.5; 1, 1;1.5, 0; 1, -1; 0, -1.5;-1,-1];

% star pentagon 
R = 1.8;
p.traj = [R*cos(pi/2),R*sin(pi/2);R*cos(pi/2+0.8*pi),R*sin(pi/2+0.8*pi);R*cos(pi/2+1.6*pi),R*sin(pi/2+1.6*pi);R*cos(pi/2+0.4*pi),R*sin(pi/2+0.4*pi);R*cos(pi/2+1.2*pi),R*sin(pi/2+1.2*pi)];

% trapezoid 
R = 0.5;
p.traj = [1*R,3*R; 3*R,2*R; 3*R,-2*R; 1*R,-3*R];

% 8 on left side
p.traj = [-1.5, 0.5; -1, 0.5; -1.5,-0.5; -1, -0.5];

% technion sign
p.traj = [0.5 0;
    0.8 0;
    1.2 -1;
    1.5 -0.3;
    1 -0.3;
    0.9 0;
    1.9 0;
    1.2 -1.5;];