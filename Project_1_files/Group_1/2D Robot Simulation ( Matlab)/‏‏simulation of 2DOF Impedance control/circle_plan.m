function [x,v,a] = circle_plan(t,Radius,center)

X=Radius*cos(t)+center(1);
Y=Radius*sin(t)+center(2);
dX=-Radius*sin(t);
dY=Radius*cos(t);
ddX=-Radius*cos(t);
ddY=-Radius*sin(t);

x=[X;Y];
v=[dX;dY];
a=[ddX;ddY];

end

