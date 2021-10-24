function [T] = DBIController(z,p,traj,iter,dt_phy)
%Controller that uses DB-IC to calculate the torque needed

if iter==1 %temporary solution to make every index excutable
    iter = 2;

end

th1 = z(1);
th2 = z(3);
thdot1 = z(2);
thdot2 = z(4);
q = [th1 th2]'; %joint value
dq = [thdot1 thdot2]'; 
x_d = traj(iter,1); %desired position
y_d = traj(iter,2);
xv_d = (traj(iter,1)-traj(iter-1,1))/dt_phy; %desired velocity
yv_d = (traj(iter,2)-traj(iter-1,2))/dt_phy;

% J  =Jacobian
J = Velocity_transformation(p.l1,p.l2,th1,th2);
J_dt = Jdt(p.l1,p.l2,th1,th2,thdot1,thdot2);
H=[p.I1 0;0 p.I2;];
Lq = ForwardKin(p.l1,p.l2,th1,th2); % forward kinematics

K = p.K; % K stiffness 
B = p.B; % B damping
M= p.M; % M inertia

F_int = [p.Fx p.Fy 0]';
    
%Torque to track our desired point

temp = K*([x_d y_d 0]'-Lq)+B*([xv_d yv_d 0]'-J*dq)-F_int;
T = H*pinv(J)*(M^-1*temp-J_dt*dq);
T = T+ -J'*F_int;

%Add gravity compensation
T(1) = T(1) + GravityCompT1(0,0,p.d1,p.d2,p.g,p.l1,p.l2,p.m1,p.m2,th1,th2,thdot1,thdot2);
T(2) = T(2) + GravityCompT2(0,0,p.d2,p.g,p.l1,p.l2,p.m2,th1,th2,thdot1);


end

