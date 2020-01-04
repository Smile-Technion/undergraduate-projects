function status = myOutPutFcn_IM(t,y,flag)

global tau tau_max
global t_Build
global kp kd kp1 kp2 kd1 kd2 Wall_d
global l1 l2 X_d a dt
global Wall_x F_in F_in_x Wall_k Xm_check X0_check
global Km Bm Mm

persistent count  % count how many calls for myOutPutFcn


switch flag
    case 'init'
        % parameter of the controle law
        kp = [kp1 0; 0 kp2];
        kd = [kd1 0; 0 kd2];
        disp('Start...')
        count = 1;
        dt = 0.001;
        
    case ''
        %         q_d = interp1(t_Build,q_theoretic',t(end))';
        %         dqd = interp1(t_Build,dq_theoretic',t(end))';
        %         dqqd = interp1(t_Build,ddq_theoretic',t(end))';
        
        q = [y(1);y(3)];
        dq = [y(2);y(4)];
        
        % dynamics matrix
        H = double(dynamics_H_new(q));
        C = double(dynamics_C_new(q,dq));
        G = double(dynamics_G_new(q));
        
        h = C*dq+G;
        
        % solve for Xm
        % initial conditions
        [Xim0_x, Xim0_y] = Forword_kinematics(q(1),q(2),l1,l2);
        Xim0_dot = Jacobian_L(q(1),q(2),l1,l2)*dq;
        Xim0_dot_x = Xim0_dot(1);
        Xim0_dot_y = Xim0_dot(2);
        X0 = interp1(t_Build, X_d(1:2,:)' ,t(end))';
        X0_dot = interp1(t_Build, X_d(3:4,:)' ,t(end))';
        
        % Wall simulation
        Griper_position = Forword_kinematics(q(1),q(2), l1, l2);
        if Griper_position(1) <= Wall_x
            F_in_x = 0;
        elseif Griper_position(1) > Wall_x
            F_in_x = abs(Wall_x - Griper_position(1))* Wall_k + Xim0_dot_x*Wall_d;
        end
        F_in = [F_in_x ;0];
        JL_T = Jacobian_L(q(1),q(2), l1, l2).';
        
        
        
        % solve Impedance Model
        options = odeset('MaxStep',1 ,'Refine',10 );  %    
        [t,Xm] = ode45(@(t,Xm) Impedance_Controller(t,Xm),[t t+dt],[Xim0_x, Xim0_y, Xim0_dot_x, Xim0_dot_y]', options);
        
        % inverse kinematics  --> q_d
        Xm = Xm';
        Xm = Xm(:,2);
        v_d = [Xm(1,1); Xm(3,1)];
        a_d = [Xm(2,1); Xm(4,1)];
        Xm = Km \ (-F_in + Km*X0 +Bm*(X0_dot-v_d)-Mm*a_d); % get position
        q_d = inv_kin(Xm(1,1),Xm(2,1),l1, l2);
        
        Xm_check = [Xm_check Xm];
        X0_check = [X0_check X0];
        
        
        for i = 1:length(2)
            dq_d(:,i)= Jacobian_inv(l1,l2,q_d(:,i)) * v_d(:,i);
            ddq_d(:,i)= Jacobian_inv(l1,l2,q_d(:,i))*(a(:,i)-Jacobian_dot(l1,l2,q_d(:,i),dq_d(:,i))*dq_d(:,i));
        end
        
        
        % control law
%         tau = -JL_T*F_in + h + H*(ddq_d +kp*(q_d-q)+kd*(dq_d-dq));
        tau = -JL_T*F_in + h + H*(ddq_d +kp*(q_d-q)+kd*(dq_d-dq));

                
        % Apply motor Torqe Limit
        tau1 = tau(1);
        tau2 = tau(2);
        % motor 1
        if tau1 > tau_max
            tau1 = tau_max;
        elseif tau1 < -tau_max
            tau1 = -tau_max;
        end
        % motor 2
        if tau2 > tau_max
            tau2 = tau_max;
        elseif tau2 < -tau_max
            tau2 = -tau_max;
        end
        tau = [tau1 ; tau2];
        
        
        count = count +1;
        
    case 'done'
        disp('Number of Controller Counts:')
        disp(count)
        disp('done.')
end

status = 0;

end

