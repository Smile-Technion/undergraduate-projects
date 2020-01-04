function Jacobian_L = Jacobian_L(theta1 ,theta2, l1, l2)

c1 = cos(theta1);
s1 = sin(theta1);
c12 = cos(theta1+theta2);
s12 = sin(theta1+theta2);


JL1 = [ -l1*s1-l2*s12 ; l1*c1+l2*c12];
JL2 = [ -l2*s12       ; l2*c12      ];

Jacobian_L = [JL1 JL2];

end

