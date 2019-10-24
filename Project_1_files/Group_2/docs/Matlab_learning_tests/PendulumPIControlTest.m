L =1;
m=1;
I = m*L^2;
g = 9.81;
dt = 1e-3;
y0 = [0 0];
T=0;

K = 50;
w0 = sqrt(g/L);
C = 1;

theta_ref = 1;
Kp = 10;
Ki = 1;
Tint = 0;
counter = 1;
t_current = 0;
t_end = 100;
MAX_T = 40;
while t_current<t_end

tspan = [t_current, t_current+dt];

[t,y] = ode45(@(t,y) odefcn(t,y,T,K,I,w0,C), tspan, y0);

y0 = [y(end,1), y(end,2)];
    if abs(Tint)<MAX_T
        Tint = Tint + Ki*Kp*dt*(theta_ref-y(end,1)); %Integrator
    end
T=(theta_ref-y(end,1))*Kp+Tint;
Fext = K*y0(1)+C*y0(2)+(w0^2)*cos(y0(1));
    if (Fext>MAX_T)
        T=0;
    end
    if T>MAX_T
        T=MAX_T;
    end
    t_current = t(end);
    T_history(counter) = T;
    theta_history(counter)  = y0(1);
    time(counter) = t_current;
    
    counter = counter+1;
end

figure
plot(time,theta_history)
title('\theta')
figure

plot(time,T_history)
title('Torque')


function dydt = odefcn(t,y,T,K,I,w0,C)
dydt = zeros(2,1);
dydt(1) = y(2);
dydt(2) = -(w0^2)*cos(y(1))-K*y(1)/I+T/I-C*y(2)/I;
end

