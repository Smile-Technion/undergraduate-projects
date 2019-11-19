clc
clear all
global m1 m2 l1 l2
m1=2;
m2=2;
l1 = 2;
l2 = 2;

syms g m1 m2 m3 m4 t1 t2 d4 t1_dot t2_dot d4_dot l1 l2 l3 l4 l5
syms t1_dot2 t2_dot2 d4_dot2 f1 f2 f3 M1 M2 M3
A01=[cos(t1) -sin(t1) 0 0; sin(t1) cos(t1) 0 0 ; 0 0 1 0 ; 0 0 0 1];
A12=[cos(t2) -sin(t2) 0 l1 ; sin(t2) cos(t2) 0 0 ; 0 0 1 0 ; 0 0 0 1];
A23=[1 0 0 l2 ; 0 1 0 0 ; 0 0 1 0 ; 0 0 0 1];


A02=A01*A12;
A06=A01*A12*A23;


R0Tool=A06(1:3,1:3);

%% jacobian
Jl=simplify(jacobian(A06(1:3,4), [t1, t2]))
JlTool=simplify((R0Tool.')*Jl)


%% JLi Tool
JLw=sym(zeros(3,2,2));
JLw(:,:,2)=subs(Jl,[l2],[l2/2]);
JLw(:,:,1)=subs(Jl,[l2 t2 l1],[0 0 l1/2]);


%% JLi word
JL=sym(zeros(3,2,2));
JL(:,:,2)=subs(JlTool,[l2],[l2/2]);
JL(:,:,1)=subs(JlTool,[l2 t2 l1 ],[0 0 l1/2]);

%% JAi Tool
JA=sym(zeros(3,2,2));
JA(:,:,2)=[0 0; 0 0 ; 1 1];
JA(:,:,1)=[0 0; 0 0 ; 1 0];


%% inertia
m=sym([m1 m2]);
I=sym(zeros(3,3,2));
I(:,:,1)=(m(1)*l1^2)/12*[0 0 0 ; 0 1 0 ; 0 0 0];
I(:,:,2)=(m(2)*(l2)^2)/12*[0 0 0 ; 0 1 0 ; 0 0 0];

%% H
H=sym(zeros(2,2));

for i=1 : 2
H=H+m(i)*(JL(:,:,i).')*JL(:,:,i)+(JA(:,:,i).')*I(:,:,i)*JA(:,:,i);
end

%% C

c=sym(zeros(2,2,2));
C=sym(zeros(2,2));


q(1,1)=t1;
q(2,1)=t2;

q_dot(1,1)=t1_dot;
q_dot(2,1)=t2_dot;

for i=1 : 2
    for j=1 :2
        for k=1 :2
          c(k,j,i)=0.5*(diff(H(i,j),q(k))+diff(H(i,k),q(j))-diff(H(j,k),q(i)));  
          C(i,j)=C(i,j)+c(k,j,i)*q_dot(k);
        end
    end
end

%% G

G=sym(zeros(2,1));
g=[0 ; 0 ; -g];
for n=1 : 2
    G=G-m(n)*(JLw(:,:,n).')*g;
end

%% tho

q_dot2(1,1)=t1_dot2;
q_dot2(2,1)=t2_dot2;
f(1,1)=f1;
f(2,1)=f2;
f(3,1)=f3;
f(4,1)=M1;
f(5,1)=M2;
f(6,1)=M3;
JaTool=[0 0; 0 0 ; 1 1];
tau=H*q_dot2+C*q_dot+G-([JlTool ; JaTool].')*f;

%%

% tau=zeros(1,length(t_vec));
%%
t_vec=linspace(0,10,500);
tau=[zeros(1,length(t_vec)); 1*ones(1,length(t_vec))];
q0=[0 0];
dq0=[0 0];
options=odeset('MaxStep',0.1);

[tm,X]=ode45(@(tm,X) state_eq_new(tm,X,t_vec,tau),t_vec,[q0(1) dq0(1) q0(2) dq0(2)]',options);


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

theta1=X(:,1)';
theta2=X(:,3)';

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

Grip = zeros(2,length(X(:,1)));  % initiate grip position log vector
hold on
grid on

for k=1:length(X(:,1))
    
    Grip(1,k) = l2_line_end(1,k); % log grip position
    Grip(2,k) = l2_line_end(2,k);
    
    set(hArm1,'XData',[0, l1_line_end(1,k)],'YData',[0 , l1_line_end(2,k)]);
    set(hArm2,'XData',[l1_line_end(1,k), l2_line_end(1,k)],'YData',[l1_line_end(2,k) , l2_line_end(2,k)]);
    set(hMark,'XData',Grip(1,k),'YData',Grip(2,k));
    drawnow
    xlim(hAxes,[-4 4]);

%     plot(Grip(1,:), Grip(2, :), '*g')
   
end
