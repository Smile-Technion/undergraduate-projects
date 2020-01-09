clear variables; close all; clc

global tau t_Build q_theoretic dq_theoretic
global kp kd kp1 kp2 kd1 kd2 g ddq_theoretic
kp1 = 50;  kp2 = 50;  kd1 = 10;  kd2 = 10;
tau = [0 0]';
g = 9.8 ; 
t_Build = linspace(0,10,1000);

% Robot Definition
l1 = 5;
l2 = 5;
m1 = 2;
m2 = 2;

% define start and end position
X0 = [2 2] ;
Xf = [-6 6] ;

% build motion plan
[X,Y, X_dot, Y_dot, X_2dot, Y_2dot] = Motion_plan(X0,Xf,t_Build);
v = [X_dot; Y_dot];
a = [X_2dot; Y_2dot];

% inverse kinematics 
q_theoretic = inv_kin(X,Y,l1,l2);

% clac joint speed and acceleration 
dq_theoretic = zeros(2, length(t_Build));
ddq_theoretic = zeros(2, length(t_Build));
tau_theoretic = zeros(2, length(t_Build));

for i = 1:length(t_Build)
    dq_theoretic(:,i)= Jacobian_inv(l1,l2,q_theoretic(:,i)) * v(:,i);
    ddq_theoretic(:,i)= Jacobian_inv(l1,l2,q_theoretic(:,i))*(a(:,i)-Jacobian_dot(l1,l2,q_theoretic(:,i),dq_theoretic(:,i))*dq_theoretic(:,i));
    tau_theoretic(:,i)= dynamics_H_new(q_theoretic(:,i)) * ddq_theoretic(:,i) + dynamics_C_new(q_theoretic(:,i),dq_theoretic(:,i))* dq_theoretic(:,i)+ dynamics_G_new(q_theoretic(:,i));
end

% check tau plot
% plot(t, tau(1,:))
% hold on
% plot(t, tau(2,:))

%%
% tau = [zeros(1,length(t_v)); 1*ones(1,length(t_v))];

% Speed Initial condition
q0 = q_theoretic(:,1)';
dq0 = dq_theoretic(:,1)';

% q0=[0 0];
% dq0=[0 0];


% options = odeset('MaxStep',0.1);  % adjust solver options  (, options)
tau_theoretic=double(tau_theoretic);

% solve for Tau without control
%  [t,y] = ode45(@(t,y) state_eq_new(t,y,t_Build,tau),[0 10],[q0(1) dq0(1) q0(2) dq0(2)]');

% solve for Tau and law control : tau = G-Kp*(q-qd)-Kd*(dq-dqd)
options = odeset('OutputFcn',@myOutPutFcnPD,'MaxStep',2, 'Refine',1);
[t,y] = ode45(@(t,y) state_eq_control(t,y,t_Build,q_theoretic,dq_theoretic,ddq_theoretic),[0 10],[q0(1) dq0(1) q0(2) dq0(2)]' , options);

% Impedance Controller
% options = odeset('OutputFcn',@myOutPutFcn_IM,'MaxStep',2, 'Refine',1);
% [t,y] = ode45(@(t,y) state_eq_control(t,y,t_Build,q_theoretic,dq_theoretic,ddq_theoretic),[0 10],[q0(1) dq0(1) q0(2) dq0(2)]' , options);

% % plot Robot Arm
%  plot_Robot(y',l1,l2,y(:,1)',y(:,3)',10,10, 0)

%% plot qoints 

figure()

subplot(2,2,1);
plot(t, y(:,1))
hold on
plot(t_Build, q_theoretic(1,:))
legend('joint 1 actual Position','joint 1 wanted Positon')
xlabel('Time [sec]')
ylabel('joint 1 [rad]')

subplot(2,2,2);
plot(t, y(:,2))
hold on
plot(t_Build, dq_theoretic(1,:))
legend('joint 1 actual speed','joint 1 wanted speed')
xlabel('Time [sec]')
ylabel('speed joint 1 [rad/sec]')

subplot(2,2,3);
plot(t_Build, q_theoretic(1,:)-interp1(t,y(:,1),t_Build))
xlabel('Time [sec]')
ylabel('Position rror [rad]')

subplot(2,2,4);
plot(t_Build, dq_theoretic(1,:)-interp1(t,y(:,2),t_Build))
xlabel('Time [sec]')
ylabel('speed error [rad/sec]')


figure()

subplot(2,2,1);
plot(t, y(:,3))
hold on
plot(t_Build, q_theoretic(2,:))
legend('joint 2 actual Position','joint 2 wanted Position')
xlabel('Time [sec]')
ylabel('joint 2 [rad]')

subplot(2,2,2);
plot(t, y(:,4))
hold on
plot(t_Build, dq_theoretic(2,:))
legend('joint 2 actual speed','joint 2 wanted speed')
xlabel('Time [sec]')
ylabel('speed joint 2 [rad/sec]')

subplot(2,2,3);
plot(t_Build, q_theoretic(2,:)-interp1(t,y(:,3),t_Build))
xlabel('Time [sec]')
ylabel('Position rror [rad]')

subplot(2,2,4);
plot(t_Build, dq_theoretic(2,:)-interp1(t,y(:,4),t_Build))
xlabel('Time [sec]')
ylabel('speed error [rad/sec]')

% Compares desirable and relayed values
% figure()
% plot(t, y(:,1))
% hold on
% plot(t_Build, q_theoretic(1,:))
% legend('y','qd1')
% 
% figure()
% plot(t, y(:,3))
% hold on
% plot(t_Build, q_theoretic(2,:))
% legend('y','qd2')
% 
% 
% figure()
% plot(t, y(:,2))
% hold on
% plot(t_Build, dq_theoretic(1,:))

