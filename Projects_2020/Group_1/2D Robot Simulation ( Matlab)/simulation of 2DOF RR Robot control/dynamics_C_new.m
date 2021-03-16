function [C] = dynamics_C_new(q,qdot)

m1=2;
m2=2;
l1 = 5;
l2 = 5;

c1 = cos(q(1));
c2 = cos(q(2));
s1 = sin(q(1));
s2 = sin(q(2));

c11 = -(l1*l2*m2*qdot(2)*s2)/2;
c12 = -0.5*(l1*l2*m2*s2)*(qdot(1)+qdot(2));
c21 = (l1*l2*m2*qdot(1)*s2)/2;
c22 = 0;

C = [c11 c12;
     c21 c22 ];

% C =... 
% [  qdot(2)*(l1^2*m2*cos(q(2))*sin(q(2)) - l1*m2*sin(q(2))*(l2/2 + l1*cos(q(2)))), qdot(1)*(l1^2*m2*cos(q(2))*sin(q(2)) - l1*m2*sin(q(2))*(l2/2 + l1*cos(q(2)))) - (l1*l2*m2*qdot(2)*sin(q(2)))/2 ; ...
%  -qdot(1)*(l1^2*m2*cos(q(2))*sin(q(2)) - l1*m2*sin(q(2))*(l2/2 + l1*cos(q(2)))),                                                                                                  0];
%  
end