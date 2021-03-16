clear all
clc
counter = 1;
t_end = 20;
k_initial = 0.005;
k = k_initial;
c = 0;
m = 1;
ref = 5;
t_current = 0;
x0 = [0 0];
dt_orig = 1e-3;
dt = dt_orig;
dt_highres = 1e-6;
F = 0;
Kp=10;
Km=500;
Bp=10;
Bm=10;
Mp=1;
Mm=1;
Kint = 0;
MAX_U = 1e4
while t_current<t_end
    xp=x0(1);           %plant position 
    vp=x0(2);           %plant velocity
    [x0t v0t] = traj(t_current);% desired trajectory
    tspan = [t_current (t_current+dt)];
    Frandom = (xp>1)*10;  % Interaction force that shows only after xp=1 
    Fint = Kint*xp +Frandom; % Kmat*(theta0-theta)+Bmat*(w0-omega);
    %control law
    U =-Kp*(xp)-Bp*(vp)-(Mp/Mm)*(Km*(x0t-xp)+Bm*(v0t-vp)-Fint)-Fint;
    %for limiting  control force
        if abs(U)>MAX_U 
          sign = (U>0)*1-(U<0)*1;
          U = MAX_U *sign;
        end
    %ode solver
    [t,x] = ode45(@(t,x)  odefcn(t,x,U,Kp,Mp,Bp,Fint), tspan, x0);
    t_current=t(end);
    xcurrent = x(end,1);
    
    %initial condition for next ode calc
    x0 = [x(end,1), x(end,2)];
    
    %save data for plotting 
    U_history(counter) = U;
    t_history(counter) = t_current;
    x_history(counter) = xcurrent;
  
    counter = counter+1;
	
end


figure
plot(t_history, x_history)
title('X');
hold on
time = 0:dt:t_end;
plot(time,traj(time))

figure
plot(t_history,U_history)
title('U')
disp('max x:')
max(x_history)
disp('max u:')
max(abs(U_history))


%ode func plant diff eq
function dxdt = odefcn(t,x,U,Kp,Mp,Bp,Fint)
dxdt = zeros(2,1);
dxdt(1) = x(2);
dxdt(2) = -(Bp/Mp)*x(2)-(Kp/Mp)*x(1)-(Fint+U)/Mp;
end

%desired trajectory func
function [xtraj, vtraj] = traj(t)

for i=1:length(t)
    xtraj(i) = 0.5*t(i);
    vtraj(i) = 0.5;
    
    if xtraj(i) >= 4
        xtraj(i) = 4;
        vtraj(i) = 0;
    end
    
end



end

