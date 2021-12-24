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
 
% Design LQR controller that stabilizes the closed loop 
K = lqr(A,B,Q,R);
disp("GAIN MATRIX:")
disp(K)

% Checking stability of closed loop system
disp("EIGEN VALUES OF CLOSED LOOP")
disp(eig(A-B*K))

% Now we need to find L that will also stabilize the state space estimation
% [(A - BK)  BK
%     0    (A-LC)]
% For this A-LC needs to be stable, have negative real part Eigen Values. To ensure
% this we do pole placement.
poles = [-1 -3 -5 -7 -9 -11];

% Now we will compute the best L for each of the 3 observable cases
% Case 1: x(t)
C1 = [1 0 0 0 0 0];

L1 = place(A', C1', poles)';

% Compute the new state space representation
A_L1 = [(A-B*K)           (B*K);
        zeros(size(A))   (A - L1*C1)];
B_L1 = [B;
        zeros(size(B))];
C_L1 = [C1 zeros(size(C1))];

Observer_ss1 = ss(A_L1, B_L1, C_L1, D);

% Case 3: x(t), theta2(t)
C3 = [1 0 0 0 0 0;
     0 0 0 0 1 0];
 
L3 = place(A', C3', poles)';

% Compute the new state space representation
A_L3 = [(A-B*K)           (B*K);
        zeros(size(A))   (A - L3*C3)];
B_L3 = [B;
        zeros(size(B))];
C_L3 = [C3 zeros(size(C3))];

Observer_ss3 = ss(A_L3, B_L3, C_L3, D);

% Case 4: x(t), theta1(t), theta2(t)
C4 = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
 
L4 = place(A', C4', poles)';

% Compute the new state space representation
A_L4 = [(A-B*K)           (B*K);
        zeros(size(A))   (A - L4*C4)];
B_L4 = [B;
        zeros(size(B))];
C_L4 = [C4 zeros(size(C4))];

Observer_ss4 = ss(A_L4, B_L4, C_L4, D);

% Set initial state conditions for estimate and real state
X_0 = [3 0 12 0 10 0 0 0 0 0 0 0];
figure
initial(Observer_ss1,X_0)
title("Initial Response Case 1")
figure
step(Observer_ss1)
title("Step Input Response Case 1")

figure
initial(Observer_ss3,X_0)
title("Initial Response Case 3")
figure
step(Observer_ss3)
title("Step Input Response Case 3")

figure
initial(Observer_ss4,X_0)
title("Initial Response Case 4")
figure
step(Observer_ss4)
title("Step Input Response Case 4")

grid on

