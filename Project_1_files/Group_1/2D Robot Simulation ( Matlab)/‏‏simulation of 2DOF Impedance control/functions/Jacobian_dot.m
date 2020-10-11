function [Jl_dot] = Jacobian_dot(l1,l2,q,dq)

c1=cos(q(1));
s1=sin(q(1));
s2=sin(q(2));
c12=cos(q(1)+q(2));
s12=sin(q(1)+q(2));

jL11= -l2*c12*(dq(1)+dq(2)) - l1*c1*dq(1);
jL12= -l2*c12*(dq(1)+dq(2));
jL21= -l2*s12*(dq(1)+dq(2)) - l1*s1*dq(1);
jL22= -l2*s12*(dq(1)+dq(2));

Jl_dot= [jL11 jL12;
         jL21 jL22];

end

