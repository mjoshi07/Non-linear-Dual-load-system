clc
clear all

M=1000; m1 = 100; m2 =100;
l1 =20; l2 =10;
g = 9.89;

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

D = 0;

% Set Q & R to tuned values
Q = [20 0 0 0 0 0;
     0 20 0 0 0 0;
     0 0 1500 0 0 0;
     0 0 0 1500 0 0;
     0 0 0 0 1500 0;
     0 0 0 0 0 1500];

 R = 0.05;
 C1 = [1 0 0 0 0 0];
 
% Design LQR controller that stabilizes the closed loop 
K = lqr(A,B,Q,R);
disp("GAIN MATRIX:")
disp(K)

% Checking stability of closed loop system
disp("EIGEN VALUES OF CLOSED LOOP")
disp(eig(A-B*K))

% Process Noise
Bd = [0.1;0.1;0.1;0.1;0.1;0.1];

% Measurement Noise
V = 1;

% Calculate the Kalman Bucy filter gain due to given process noise
state_space = ss(A, [B Bd],C1,0);
R = 0.01; Q= 0.05;
sensors = [1];
W = [1];
[~,L_gauss,~] = kalman(state_space, Q, R, [], sensors, W);

disp("Kalman Bucy Filter Gains");
disp(L_gauss);

A_L1 = [(A-B*K)           (B*K);
        zeros(size(A))   (A - L_gauss*C1)];
B_L1 = [B;
        zeros(size(B))];
C_L1 = [C1 zeros(size(C1))];

state_space2 = ss(A_L1, B_L1, C_L1, D);

X_0 = [3 0 12 0 10 0 0 0 0 0 0 0];
figure
initial(state_space2,X_0)
title("Initial Response Case 1")
figure
step(state_space2)
title("Step Input Response Case 1")