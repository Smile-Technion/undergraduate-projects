clear all; close all; clc
m1 =2;
m2 =1;
l1 = 2;
l2 = 1;
i1 = m1*l1^2/12;    %  in of Rod
i2 = m2*l2^2/12; 

syms theta1 theta2
c2 = cos(theta2);
s2 = sin(theta2);

JL2 = [l1*s2,          0;
       l2*0.5-l1*c2,   l2*0.5;
       0           ,   0];
   
JL1 = [0,        0;
       -l1*0.5,  0;
       0     ,   0];
   
JA2 = [0    0;
       0,   0;
       1,   1];
   
JA1 = [0    0;
       0,   0;
       1,   0];

I1 = [0 0 0; 0 i1 0; 0 0 0];
I2 = [0 0 0; 0 i2 0; 0 0 0];

H = m1*JL1'*JL1 + m2*JL2'*JL2 + JA1'*I1*JA1 + JA2'*I2*JA2;


      