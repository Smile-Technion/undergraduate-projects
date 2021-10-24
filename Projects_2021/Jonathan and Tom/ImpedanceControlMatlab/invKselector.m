function [SELECT] = invKselector(invksol, sol_select, pinit)
% there are two solutions for the inverse kinematics
% SELECT returns 1 or 2 depends on the solution number
% It will choose the solution which provides continuous motion
if sol_select(1)*sol_select(2)*sol_select(3)==0
    d1 = abs(mod(pinit(1)  - invksol(3,1),2*pi));
    d2 = abs(mod(pinit(1)  - invksol(3,2),2*pi));
    if d1>d2
        SELECT = 2;
        disp('2 selected')
    else
        SELECT = 1;
        disp('1 selected')
    end
else 
    q1_prev = [invksol(1,sol_select), invksol(2,sol_select)];
    q2_prev = [invksol(1,sol_select+2), invksol(2,sol_select+2)];
    q_accelration1 = [(q1_prev(1)-2*q1_prev(2)+invksol(3,1)), (q2_prev(1)-2*q2_prev(2)+invksol(3,3))];
    q_accelration2 = [(q1_prev(1)-2*q1_prev(2)+invksol(3,2)), (q2_prev(1)-2*q2_prev(2)+invksol(3,4))];
    
    if sum(q_accelration1) > sum(q_accelration2)
        SELECT = 2;
    else
        SELECT = 1;
    end
    
end
end

