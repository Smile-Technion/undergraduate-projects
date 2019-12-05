clear variables; close all; clc

t_v = linspace(0,10,100);

% Robot Definition
l1 = 5;
l2 = 5;
m1 = 2;
m2 = 2;

<<<<<<< HEAD

% inverse Linear Jacobian
inv_JL=@(t1,t2) [ -1./(l2.*sin(t1 + t2) + l1.*sin(t1)), -1./(l2.*sin(t1 + t2));  1./(l2.*cos(t1 + t2) + l1.*cos(t1)),  1./(l2.*cos(t1 + t2))];

dJL=@(t1,t2,dt1,dt2) [ - l2.*cos(t1 + t2).*(dt1 + dt2) - l1.*cos(t1).*dt1, -l2.*cos(t1 + t2).*(dt1 + dt2);
 - l2.*sin(t1 + t2).*(dt1 + dt2) - l1.*sin(t1).*dt1, -l2.*sin(t1 + t2).*(dt1 + dt2)];

=======
>>>>>>> 646cf5e15e4018c4254274e2b02f36847fcd89c4
% define start and end position
X0 = [2 2] ;
Xf = [6 6] ;

% build motion plan
[X,Y, X_dot, Y_dot, X_2dot, Y_2dot] = Motion_plan(X0,Xf,t_v);
v = [X_dot; Y_dot];
a = [X_2dot; Y_2dot];

% inverse kinematics 
q = inv_kin(X,Y,l1,l2);

% clac joint speed and acceleration 
dq_theoretic = zeros(2, length(t_v));
ddq_theoretic = zeros(2, length(t_v));
tau = zeros(2, length(t_v));

for i = 1:length(t_v)
    dq_theoretic(:,i)= Jacobian_inv(l1,l2,q(:,i)) * v(:,i);
    ddq_theoretic(:,i)= Jacobian_inv(l1,l2,q(:,i))*(a(:,i)-Jacobian_dot(l1,l2,q(:,i),dq_theoretic(:,i))*dq_theoretic(:,i));
    tau(:,i)= dynamics_H_new(q(:,i)) * ddq_theoretic(:,i) + dynamics_C_new(q(:,i),dq_theoretic(:,i))* dq_theoretic(:,i)+ dynamics_G_new(q(:,i));
end

% check tau plot
% plot(t, tau(1,:))
% hold on
% plot(t, tau(2,:))

%%
% tau = [zeros(1,length(t_v)); 1*ones(1,length(t_v))];

% Speed Initial condition
q0 = q(:,1)';
dq0 = dq_theoretic(:,1)';

% q0=[0 0];
% dq0=[0 0];


options = odeset('MaxStep',0.1);  % adjust solver options
tau=double(tau);

% solve for Tau
% [t,y] = ode45(@(t,y) state_eq_new(t,y,t_v,tau),t_v,[q0(1) dq0(1) q0(2) dq0(2)]', options);

% solve for Tau and law control  tau = G-Kp*(q-qd)-Kd*(dq-dqd)
[t,y] = ode45(@(t,y) state_eq_control(t,y,t_v,q,dq_theoretic,ddq_theoretic),t_v,[q0(1) dq0(1) q0(2) dq0(2)]', options);

% Compares desirable and relayed values
figure()
plot(t, y(:,1))
hold on
plot(t_v, q(1,:))
legend('y','qd1')

figure()
plot(t, y(:,3))
hold on
plot(t_v, q(2,:))
legend('y','qd2')


figure()
plot(t, y(:,2))
hold on
plot(t_v, dq_theoretic(1,:))

% plot_Robot(q,l1,l2,q(1,:),q(2,:),10,10,0)
plot_Robot(y',l1,l2,y(:,1)',y(:,3)',10,10,1)
