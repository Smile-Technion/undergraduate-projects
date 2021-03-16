function [Griper_x,Griper_y] = Forword_kinematics(theta1, theta2, l1, l2)
% this function outputs the End Effector's position base on joint angle

c1 = cos(theta1);
s1 = sin(theta1);
c12 = cos(theta1+theta2);
s12 = sin(theta1+theta2);

l1_line_end = [l1.*c1; l1.*s1];
l2_line_end = [l2.*c12 ; l2.*s12]+l1_line_end;

Griper_x = l2_line_end(1);
Griper_y = l2_line_end(2);

end

