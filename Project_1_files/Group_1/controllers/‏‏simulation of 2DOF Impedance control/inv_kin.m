function q = inv_kin(px,py,l1,l2)

c2 = (px.^2 + py.^2 - l1^2 - l2^2)./(2*l1*l2) ; 
s2 = sqrt(1-c2.^2);
theta2 = atan2(s2, c2);

k1 = l1+l2*c2;
k2 = l2*s2;

c1 = (k1.*px + k2.*py)./ (k1.^2+k2.^2) ;   
s1 = (-k2.*px + k1.*py)./ (k1.^2+k2.^2) ;
theta1 = atan2(s1, c1);

q = [theta1 ; theta2];
end

