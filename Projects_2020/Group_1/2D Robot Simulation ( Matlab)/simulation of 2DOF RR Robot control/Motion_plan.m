function [X,Y, X_dot, Y_dot, X_2dot, Y_2dot] = Motion_plan(X0,Xf,t)

T = t(end);
a0 = X0(1);
a3 = 10*(Xf(1)- X0(1))/ T^3 ;
a4 = -15*(Xf(1)- X0(1))/ T^4 ;
a5 = 6*(Xf(1)- X0(1))/ T^5 ;

X = a5*t.^5 + a4*t.^4 + a3*t.^3 + a0 ;

m = (Xf(2)-X0(2))/(Xf(1)- X0(1)) ;
n = (Xf(1)*X0(2)- X0(1)*Xf(2)) / (Xf(1)-X0(1));
Y = m*X + n ;

X_dot = 5*a5*t.^4 + 4*a4*t.^3 + 3*a3*t.^2 ;
Y_dot = m*X_dot;

X_2dot = 20*a5*t.^3 + 12*a4*t.^2 + 6*a3*t ;
Y_2dot = m*X_2dot;

end

