function [dydt]=state_eq_new(t,y, our_time,tau)
%t_vec and tau are pre-defined!

tau_interp = [interp1(our_time,tau(1,:),t);interp1(our_time,tau(2,:),t)];

q = [y(1) ; y(3)];
dq = [y(2) ; y(4)];

H = double(dynamics_H_new(q));
C = double(dynamics_C_new(q,dq));
G = double(dynamics_G_new(q));

Dynamics = H\(-C*dq -G + tau_interp);

dydt = [y(2) ; Dynamics(1) ; y(4) ; Dynamics(2)];


% tau_interp = [interp1(our_time,tau(1,:),t);interp1(our_time,tau(2,:),t)];
% plot(t,tau(1,:),'o',t,interp1(t,tau(1,:),tm),':.');
% hold on
% 
% q = [y(1);y(3)];
% dq = [y(2);y(4)];
% 
% H = double(dynamics_H_new(q));
% C = double(dynamics_C_new(q,dq));
% G = double(dynamics_G_new(q));
% 
% ddq = inv(H)*(-C*dq-G+tau_interp);
% 
% dydt = [dq(1);ddq(1);dq(2);ddq(2)];
% % Xdot=double(Xdot);
end