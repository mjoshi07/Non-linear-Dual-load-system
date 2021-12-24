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
 
% Checking Observality for the given cases
% Case 1: x(t)
C = [1 0 0 0 0 0];

% Check rank for observability_matrix
observability_matrix = [C' A'*C' (A'^2)*C' (A'^3)*C' (A'^4)*C' (A'^5)*C'];
if(rank(observability_matrix)==6)
    disp("For Case 1: x(t) ====> Observable")
else
    disp("For Case 1: x(t) ====> Non Observable")
end

% Case 2: theta1(t), theta2(t)
C = [0 0 1 0 0 0;
     0 0 0 0 1 0];

% Check rank for observability_matrix
observability_matrix = [C' A'*C' (A'^2)*C' (A'^3)*C' (A'^4)*C' (A'^5)*C'];
if(rank(observability_matrix)==6)
    disp("For Case 1: theta1, theta2 ====> Observable")
else
    disp("For Case 1: theta1, theta2 ====> Non Observable")
end

% Case 3: x(t), theta2(t)
C = [1 0 0 0 0 0;
     0 0 0 0 1 0];

% Check rank for observability_matrix
observability_matrix = [C' A'*C' (A'^2)*C' (A'^3)*C' (A'^4)*C' (A'^5)*C'];
if(rank(observability_matrix)==6)
    disp("For Case 1: x(t), theta2(t) ====> Observable")
else
    disp("For Case 1:x(t), theta2(t)====> Non Observable")
end

% Case 4: x(t), theta1(t), theta2(t)
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];

% Check rank for observability_matrix
observability_matrix = [C' A'*C' (A'^2)*C' (A'^3)*C' (A'^4)*C' (A'^5)*C'];
if(rank(observability_matrix)==6)
    disp("For Case 1: x(t), theta1(t), theta2(t) ====> Observable")
else
    disp("For Case 1: x(t), theta1(t), theta2(t) ====> Non Observable")
end