function J = Jdt(l1,l2,th1,th2,th1d,th2d)
% Robot Jacobian time derivative
% th1d, th2d = joint value time derivative
thd = th1d+th2d;

J=[l2*thd*sin(th1+th2)+l1*sin(th1)*th1d, l2*thd*sin(th1+th2);
    -l2*thd*cos(th1+th2)-l1*sin(th1)*th1d, -l2*thd*cos(th1+th2);
    0, 0;];