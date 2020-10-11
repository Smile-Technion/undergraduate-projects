function [Jl] = simbolic_dimanic()
global m1 m2 l1 l2
% m1=2;
% m2=2;
% l1 = 5;
% l2 = 5;

syms g  m3 m4 t1 t2 d4 t1_dot t2_dot d4_dot  l3 l4 l5 dt1 dt2 l1 l2 m1 m2
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

%% JLi world
JLw=sym(zeros(3,2,2));
JLw(:,:,2)=subs(Jl,[l2],[l2/2]);
JLw(:,:,1)=subs(Jl,[l2 t2 l1],[0 0 l1/2]);

%% JLi Tool
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
g=[0 ; -g ; 0];
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

end

