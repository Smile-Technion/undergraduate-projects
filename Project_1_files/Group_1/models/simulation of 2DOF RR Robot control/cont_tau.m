function [tau] = cont_tau(t,q,dq,t_vec,qd,dqd)

% parameter of the controle law
kp1 = 10;
kp2 = 10;
kd1 = 20;
kd2 = 20;

Kp = [kp1 0; 0 kp2];
Kd = [kd1 0; 0 kd2];


qd = interp1(t_vec,qd',t)';
dqd = interp1(t_vec,dqd',t)';

% dinamice matrix
H = double(dynamics_H_new(q));
C = double(dynamics_C_new(q,dq));
G = double(dynamics_G_new(q));

% controle law
tau = G-Kp*(q-qd)-Kd*(dq-dqd);
end

