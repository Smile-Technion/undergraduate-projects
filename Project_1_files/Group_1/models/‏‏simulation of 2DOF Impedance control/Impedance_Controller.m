function [X_m] = Impedance_Controller(t,x)

global Km Bm Mm F_in X_d t_Build 

% Get Desired path at Time point
X0 = interp1(t_Build, X_d(1:2,:)' ,t(end))';
X0_dot = interp1(t_Build, X_d(3:4,:)' ,t(end))';

Xm = [x(1); x(2)];
Xm_dot = [x(3); x(4)];

X_m_2dot = Mm\ (-F_in + Km*(X0-Xm) + Bm*(X0_dot-Xm_dot));

X_m = [Xm_dot(1); X_m_2dot(1); Xm_dot(2); X_m_2dot(2)];

end




% X0_2dot = interp1(t_Build, X_d(5:6,:)' ,t(end))';
% Xm_dot_next = Xm_dot + Mm \ ((Bm*X0_dot+Km*X0-F_in)*dt - (Bm+0.5*Km)*Xm_next+(Bm+0.5*Km)*Xm);
% X0_2dot = Mm\ (-F_in + Km*(X0-Xm) + Bm*(X0_dot-Xm_dot));
% X_m = [Xm_dot_next];
% X_m = [Xm_dot(1); X_m_2dot(1); Xm_dot(2); X_m_2dot(2)];