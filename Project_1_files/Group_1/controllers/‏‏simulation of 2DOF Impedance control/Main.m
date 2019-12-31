clear variables; close all; clc
global tau tau_max
global t_Build q_theoretic dq_theoretic ddq_theoretic
global kp kd kp1 kp2 kd1 kd2
global l1 l2 m1 m2 X0 Xf X_d
global Wall_x F_in F_in_x Wall_k Xm_check X0_check
global Km Bm Mm a

% --------Simulation Parameters -------------------

Wall_x = 5;  % Define the position of the wall
Wall_k = 150; % Define the stiffness of the wall
Window_Limit_X = 10;
Window_Limit_Y = 10;
Animation = true;
Video_on = true;
Wall_on = true;

% Robot Definition
l1 = 5;  l2 = 5;  % Length of Links
m1 = 2;  m2 = 2;  % Mass of Links
tau_max = 120; % max Torqe each motor can generate
kp1 = 50;  kp2 = 50;  % P control parameters
kd1 = 10;  kd2 = 10;    % D control parameters

% Impedane Desired Values
Km = eye(2)*7;
Bm = eye(2)*1.7;
Mm = eye(2)*0.3;

% define start and end position
X0 = [2 0] ;
Xf = [5.5 0] ;

% -------------------------------------------------

Xm_check = X0';
X0_check = X0';

tau = [0 0]';
F_in = [0 ;0];
t_Build = linspace(0,10,100);

% build motion plan
[X,Y, X_dot, Y_dot, X_2dot, Y_2dot] = Motion_plan(X0,Xf,t_Build);
Position = [X; Y];
v = [X_dot; Y_dot];
a = [X_2dot; Y_2dot];
X_d = [Position ; v; a];

% plot(t_Build, X)
% hold on
% plot(t_Build, Y)

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

% Speed Initial condition
q0 = q_theoretic(:,1)';
dq0 = dq_theoretic(:,1)';

% options = odeset('MaxStep',0.1);  % adjust solver options  (, options)
tau_theoretic = double(tau_theoretic);

% solve for Tau without control
% [t,y] = ode45(@(t,y) state_eq_new(t,y,t_Build,tau),t_Build,[q0(1) dq0(1) q0(2) dq0(2)]');

% % solve for Tau and law control : tau = G-Kp*(q-qd)-Kd*(dq-dqd)
% options = odeset('OutputFcn',@myOutPutFcnPD,'MaxStep',2, 'Refine',1);
% [t,y] = ode45(@(t,y) state_eq_control(t,y,t_Build,q_theoretic,dq_theoretic,ddq_theoretic),[0 10],[q0(1) dq0(1) q0(2) dq0(2)]' , options);

% Impedance Controller
options = odeset('OutputFcn',@myOutPutFcn_IM,'MaxStep',2, 'Refine',1);
[t,y] = ode45(@(t,y) state_eq_control(t,y),[0 10],[q0(1) dq0(1) q0(2) dq0(2)]' , options);


% plot Robot Arm
if Animation == 1
    plot_Robot(y',l1,l2,y(:,1)',y(:,3)',Window_Limit_X,Window_Limit_Y,Wall_on,Video_on)
end

% Check End Error
[final_x, final_y] = Forword_kinematics(y(end,1),y(end,3),l1,l2);
Error = X0_check(:,end) - [final_x; final_y]

%% plot Joints
figure()
plot(t, y(:,1))
hold on
plot(t_Build, q_theoretic(1,:))
legend('joint 1 actual Position','q1 wanted Positon')

figure()
plot(t, y(:,3))
hold on
plot(t_Build, q_theoretic(2,:))
legend('joint 2 actual Position','q2 wanted Position')


figure()
plot(t, y(:,2))
hold on
plot(t_Build, dq_theoretic(1,:))
legend('joint 1 actual speed','qd1')
