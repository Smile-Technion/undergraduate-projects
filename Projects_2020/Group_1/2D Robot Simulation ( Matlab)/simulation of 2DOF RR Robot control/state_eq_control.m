function [Xdot]=state_eq_control(t,y,t_Build,qd,dqd,ddqd) 
global tau
q=[y(1);y(3)];
dq=[y(2);y(4)];
% dinamice matrix
H = double(dynamics_H_new(q));
C = double(dynamics_C_new(q,dq));
G = double(dynamics_G_new(q));
% calcolate the corrected tau after the control law

% tau = cont_tau(t,q,dq,t_Build,qd,dqd);
ddq=H\(-C*dq-G + tau);

Xdot=[dq(1);ddq(1);dq(2);ddq(2)];

end