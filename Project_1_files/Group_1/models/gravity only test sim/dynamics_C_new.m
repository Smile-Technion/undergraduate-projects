function [C] = dynamics_C_new(q,qdot)

global m1 m2 l1 l2
m1=20;
m2=2;

l1 = 2;
l2 = 2;


C =... 
[  qdot(2)*(l1^2*m2*cos(q(2))*sin(q(2)) - l1*m2*sin(q(2))*(l2/2 + l1*cos(q(2)))), qdot(1)*(l1^2*m2*cos(q(2))*sin(q(2)) - l1*m2*sin(q(2))*(l2/2 + l1*cos(q(2)))) - (l1*l2*m2*qdot(2)*sin(q(2)))/2 ; ...
 -qdot(1)*(l1^2*m2*cos(q(2))*sin(q(2)) - l1*m2*sin(q(2))*(l2/2 + l1*cos(q(2)))),                                                                                                  0];
 
end