function status = myOutPutFcn(t,y,flag)
global tau t_Build q_theoretic dq_theoretic
global kp kd kp1 kp2 kd1 kd2

switch flag
    case 'init'
        % parameter of the controle law
        kp = [kp1 0; 0 kp2];
        kd = [kd1 0; 0 kd2];
%         print('Start...')
       
    case ''
        q_d = interp1(t_Build,q_theoretic',t(end))';
        dqd = interp1(t_Build,dq_theoretic',t(end))';
        
        q=[y(1);y(3)];
        dq=[y(2);y(4)];
        
        % dynamics matrix
%         H = double(dynamics_H_new(q));
%         C = double(dynamics_C_new(q,dq));
        G = double(dynamics_G_new(q));
        
        % controle law
        tau = G - kp*(q-q_d)-kd*(dq-dqd);
        
    case 'done'
%         print('done')
end

status = 0;

end

