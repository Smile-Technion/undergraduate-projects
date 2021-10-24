%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Derive double pendulum dynamics %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Derive double pendulum equations of motion, write to Thdotdot1 and
%Thdotdot2 matlab function files. THIS VERSION DERIVES IT WITH THE ELBOW
%ANGLE RELATIVE TO THE UPPER ARM ANGLE.

%Parameters symbolically.
syms m1 m2 I1 I2 g l1 l2 d1 d2 th1 th2 thdot1 thdot2 thdotdot1 thdotdot2 er1 er2 eth1 eth2 T1 T2 real

%Unit vectors, cartesian
i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';

%Rotating reference frames
er1 = [-sin(th1), cos(th1), 0]'; %Just changed it so 0 is upright.
er2 = [-sin(th2+th1), cos(th2+th1), 0]';

eth1 = [-cos(th1),-sin(th1),0]';
eth2 = [-cos(th2+th1),-sin(th2+th1),0]';

%Vectors to significant points
ra_c1 = d1*er1; %A is fixed point, B is the elbow, c1 and c2 are COMs, e end effector
rb_c2 = d2*er2;
rb_e = l2*er2;
ra_b = l1*er1;
ra_c2 = ra_b + rb_c2;
ra_e = ra_b + rb_e;

matlabFunction(ra_e, 'file', 'ForwardKin');

%Velocities
Vc1 = d1*thdot1*eth1;
VB = l1*thdot1*eth1;
Vc2 = VB + d2*(thdot2+thdot1)*eth2;
Ve = VB + l2*(thdot2+thdot1)*eth2;

%Accelerations
Ac1 = d1*thdotdot1*eth1 - d1*thdot1^2*er1;
AB = l1*thdotdot1*eth1 - l1*thdot1^2*er1;
Ac2 = d2*(thdotdot2+thdotdot1)*eth2 - d2*(thdot1 + thdot2)^2*er2  + AB;

%Force at end effector
syms Fdx Fdy real
Fd = [Fdx, Fdy, 0]';

%AMB for just link 2:
M_B = cross(rb_c2,-m2*g*j)+T2*k     + cross(rb_e,Fd); %Last term is force at end effector.
Hdot2 = I2*(thdotdot2+thdotdot1)*k + cross(rb_c2, m2*Ac2);

eqn2forthdotdot2 = solve(dot(Hdot2 - M_B,k),thdotdot2);

%AMB for whole thing:
M_A = cross(ra_c2,-m2*g*j) + cross(ra_c1,-m1*g*j) + T1*k + cross(ra_e,Fd); %Gravity for both, plus a control torque. Last term is force at end effector
Hdot1 = I2*(thdotdot2+thdotdot1)*k + cross(ra_c2, m2*Ac2) + I1*thdotdot1*k + cross(ra_c1, m1*Ac1);

eqn1forthdotdot1 = solve(dot(Hdot1 - M_A,k),thdotdot1);

%One equation for thdotdot1, one for thdotdot2.
eqn2 = simplify(solve(subs(eqn2forthdotdot2, thdotdot1, eqn1forthdotdot1)-thdotdot2,thdotdot2));
eqn1 = simplify(solve(subs(eqn1forthdotdot1, thdotdot2, eqn2forthdotdot2)-thdotdot1,thdotdot1));

%Create matlab functions for thdotdot1 and 2:
matlabFunction(eqn1, 'file', 'Thdotdot1');
matlabFunction(eqn2, 'file', 'Thdotdot2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Gravity Compensation %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T2Eq = simplify(solve((solve(eqn1,T1)-solve(eqn2,T1)),T2));

T1Eq = simplify(subs(solve(eqn1,T1),T2,T2Eq));

matlabFunction(T1Eq, 'file', 'GravityCompT1');
matlabFunction(T2Eq, 'file', 'GravityCompT2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Impedance control?   %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Jacobian relating end effector velocity to joint space vel
% ie Ve = J*qv
J = jacobian(Ve,[thdot1 thdot2]);
matlabFunction(J, 'file', 'Velocity_transformation');




% Inverse Kinematics
syms l1 l2 x0 y0 q1 q2 c1 c2 s1 s2 c12 s12 q12 real
c2 = (x0^2+y0^2-l1^2-l2^2)/(2*l1*l2);
s2 = [sqrt(1-c2^2), -sqrt(1-c2^2)];
q2 = [atan2(s2(1),c2), atan2(s2(2),c2)];
c1s1 = [l1+l2*cos(q2(1)), -l2*sin(q2(1));l2*sin(q2(1)), l1+l2*cos(q2(1))]\[x0; y0];
q1 = atan2(c1s1(2),c1s1(1));
c1s1 = [l1+l2*cos(q2(2)), -l2*sin(q2(2));l2*sin(q2(2)), l1+l2*cos(q2(2))]\[x0; y0];
q1 = [q1, atan2(c1s1(2),c1s1(1))];
matlabFunction(q1, q2, 'file', 'InverseKin');



