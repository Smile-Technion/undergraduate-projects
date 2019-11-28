function [tau] = cont_tau(t,q,dq,t_vec,qd,dqd)

kp1=10;
kp2=10;
kp3=10;

Kp=[kp1 0 0; 0 kp2 0 ; 0 0 kp3];
Kd=[kd1 0 0; 0 kd2 0 ; 0 0 kd3];

qd=interp1(t_vec,qd',t)';
dqd=interp1(t_vec,dqd',t)';

H=double(dynamics_H_new(q));
C=double(dynamics_C_new(q,dq));
G=double(dynamics_G_new(q));

tau=G-Kp*(q-qd)-Kd*(dq-dqd);
end

