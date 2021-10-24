function [T, x_m, xd_m] = PBIController(z, p, x_m, xd_m, x_0, xd_0, dt_phy)
%Controller that uses PB-IC to calculate the torque needed
%dt_phy: time step of the physical system
%x_m,xd_m: previous impedance model value as a input and updated value
%as a output.
%x_0, xd_0: virtual value
th1 = z(1);
th2 = z(3);
thdot1 = z(2);
thdot2 = z(4);

J = Velocity_transformation(p.l1,p.l2,th1,th2); % J  =Jacobian
J_dt = Jdt(p.l1,p.l2,th1,th2,thdot1,thdot2);
H=[p.I1 0;0 p.I2;];

F_int = [p.Fx, p.Fy];
K = p.K; % K stiffness for impedance model
B = p.B; % B damping for impedance model
M= p.M; % M inertia for impedance model


% intergration for the desired model
%%%% INTEGRATION %%%%%%%%%
%calculate xdd
xdd_m = (K*(x_0 - x_m) + B*(xd_0 - xd_m) - F_int)/M;
%Update x,v,a
xd_m = xd_m + xdd_m*dt_phy; %Update velocity based on old RHS call
x_m = x_m + xd_m*dt_phy;
%%%%%%%%%%%%%%%%%%%%
    
 
[q1_d, q2_d] = InverseKin(p.l1, p.l2, x_m(1), x_m(2)); % joint value desired  
solution_select = p.invKsol; % TODO: write a code that select the inverseK solution set
q_d = [q1_d(solution_select)-pi/2; q2_d(solution_select)];
qdt_d = pinv(J)*[xd_m 0]';% joint value desired derivative 
qddt_d =pinv(J)*([xdd_m 0]'-J_dt*qdt_d);% joint value desired second derivative 


% Torque to track our desired point
qdt_diff = qdt_d - [thdot1 thdot2]';
q_diff = mod(q_d - [th1 th2]', 2*pi);
for i = 1:length(q_diff)
    q_diff(i) = mod(q_diff(i), 2*pi);
    if q_diff(i) > pi
        q_diff(i) = q_diff(i) - 2*pi;
    end
end

T = H*(qddt_d + p.Kd*qdt_diff + p.Kp*q_diff);
T = T + -J'*[F_int 0]';

% Add gravity compensation , h(q)
T(1) = T(1) + GravityCompT1(0,0,p.d1,p.d2,p.g,p.l1,p.l2,p.m1,p.m2,th1,th2,thdot1,thdot2);
T(2) = T(2) + GravityCompT2(0,0,p.d2,p.g,p.l1,p.l2,p.m2,th1,th2,thdot1);
end