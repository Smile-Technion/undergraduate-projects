function [H] = dynamics_H_new(q)
global m1 m2 l1 l2
m1=2;
m2=2;

l1 = 2;
l2 = 2;


H =... 
[ (l1^2*m1)/4 + m2*(l2/2 + l1*cos(q(2)))^2 + l1^2*m2*sin(q(2))^2, (l2*m2*(l2/2 + l1*cos(q(2))))/2 ; ...
                              (l2*m2*(l2/2 + l1*cos(q(2))))/2,                   (l2^2*m2)/4];

end
