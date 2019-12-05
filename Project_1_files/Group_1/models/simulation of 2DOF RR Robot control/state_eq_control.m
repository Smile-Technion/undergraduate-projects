function [Xdot]=state_eq_control(t,X,t_vec,qd,dqd,ddqd) 

q=[X(1);X(3)];
dq=[X(2);X(4)];
% dinamice matrix
H = double(dynamics_H_new(q));
C = double(dynamics_C_new(q,dq));
G = double(dynamics_G_new(q));
% calcolate the corrected tau after the control law

tau=cont_tau(t,q,dq,t_vec,qd,dqd);
ddq=H\(-C*dq-G+tau);

Xdot=[dq(1);ddq(1);dq(2);ddq(2)];

end