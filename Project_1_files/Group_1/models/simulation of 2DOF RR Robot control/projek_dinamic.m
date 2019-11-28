clc
clear all

%%
t=linspace(0,10,100);
[path,v,a] = sircle_plan(t,Radius,senter);

inv_JL=@(t1,t2) [ -1./(l2.*sin(t1 + t2) + l1.*sin(t1)), -1./(l2.*sin(t1 + t2));  1./(l2.*cos(t1 + t2) + l1.*cos(t1)),  1./(l2.*cos(t1 + t2))];
dJL=@(t1,t2,dt1,dt2) [ - l2.*cos(t1 + t2).*(dt1 + dt2) - l1.*cos(t1).*dt1, -l2.*cos(t1 + t2).*(dt1 + dt2);
 - l2.*sin(t1 + t2).*(dt1 + dt2) - l1.*sin(t1).*dt1, -l2.*sin(t1 + t2).*(dt1 + dt2)];

q = inv_kin(path,t);
for i=1 : length(t)
    dqd(:,i)=inv_JL(q(1,i),q(2,i))*v(:,i);
    ddqd(:,i)=inv_JL(q(1,i),q(2,i))*(a(i)-dJL(q(1,i),q(2,i),dqd(1,i),dqd(2,i))*dqd(:,i));
    tau(:,i)=dynamics_H_new(q(:,i))*ddqd(:,i)+dynamics_C_new(q(:,i),dqd(:,i))*dqd(:,i)+dynamics_G_new(1);
end


%%
t_vec=linspace(0,10,10);
tau1=[zeros(1,length(t_vec)); 1*ones(1,length(t_vec))];
q0=q(:,1)';
dq0=dqd(:,1)';
% q0=[0 0];
% dq0=[0 0];
options=odeset('MaxStep',0.1);
tau=double(tau);
[tm,X1]=ode45(@(tm,X1) state_eq_new(tm,X1,t_vec,tau),t_vec,[q0(1) dq0(1) q0(2) dq0(2)]',options);


%%

dt = 0.05;     % time diff
t = dt:dt:6.5;   % time vector

% circle
% px = 1.5+cos(t);    % required x position 
% py = -1.5+sin(t);    % required y position
% plot(px,py)     % check path

% % arm extention
% px = linspace(1/sqrt(2), 3/sqrt(2), length(t));    % required x position 
% py = linspace(1/sqrt(2), 3/sqrt(2), length(t));    % required y position

%inverse kinematics
% c2 = (px.^2 + py.^2 - l1^2 - l2^2)./(2*l1*l2) ; 
% s2 = sqrt(1-c2.^2);
% theta2 = atan2(s2, c2);
% 
% k1 = l1+l2*c2;
% k2 = l2*s2;
% 
% c1 = (k1.*px + k2.*py)./ (k1.^2+k2.^2) ;   
% s1 = (-k2.*px + k1.*py)./ (k1.^2+k2.^2) ;
%  theta1 = atan2(s1, c1);

theta1=X1(:,1)';
theta2=X1(:,3)';

c1=cos(theta1);
c2=cos(theta2);
s1=sin(theta1);
s2=sin(theta2);

%% plot 
hFig = figure(1);
hAxes = axes(hFig);
hold on
xlim(hAxes,[-4 4]);
ylim(hAxes,[-4 4]);
grid on
axis equal
hMark = plot(nan,nan,'ro','MarkerSize',7);
hArm1 = line([nan,nan],[nan,nan],'LineWidth',3);
set(hArm1, 'color', 'k')
hArm2 = line([nan,nan],[nan,nan],'LineWidth',3);

l1_line_end = [l1.*c1; l1.*s1];
l2_line_end = [l2.*cos(theta1+theta2) ; l2.*sin(theta1+theta2)]+l1_line_end;

Grip = zeros(2,length(X1(:,1)));  % initiate grip position log vector
hold on
grid on

for k=1:length(X1(:,1))
    
    Grip(1,k) = l2_line_end(1,k); % log grip position
    Grip(2,k) = l2_line_end(2,k);
    
    set(hArm1,'XData',[0, l1_line_end(1,k)],'YData',[0 , l1_line_end(2,k)]);
    set(hArm2,'XData',[l1_line_end(1,k), l2_line_end(1,k)],'YData',[l1_line_end(2,k) , l2_line_end(2,k)]);
    set(hMark,'XData',Grip(1,k),'YData',Grip(2,k));
    drawnow
    xlim(hAxes,[-4 4]);
    pause(0.5)
%     plot(Grip(1,:), Grip(2, :), '*g')
   
end
