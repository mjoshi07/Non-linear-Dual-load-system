clc
clear all

% Define the constant variables
syms g m1 m2 l1 l2 M

% Form the State space representation : X_dot(t) = AX(t) + BU(t)
A = [0 1                    0 0                  0  0;
     0 0            (-g*m1)/M 0          (-g*m2)/M  0;
     0 0                    0 1                  0  0;
     0 0   (-g*(m1+M))/(M*l1) 0     (-g*m2)/(M*l1)  0;
     0 0                    0 0                  0  1;
     0 0       (-g*m1)/(M*l2) 0 (-g*(m2+M))/(M*l2)  0];
     
B = [0;
     1/M;
     0;
     1/(M*l1);
     0;
     1/(M*l2)];
 
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];

Controllability_matrix = [B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B];
Controllability_matrix
rank(Controllability_matrix)
det(Controllability_matrix)