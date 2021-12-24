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
 
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];

D = 0;

% Check Controllability first
Controllability_matrix = [B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B];
if(rank(Controllability_matrix) == 6)
    disp("A full rank controllability pair found, now designing an LQR Controller for it")
else
    disp("System is not controllable, exit the system")
    exit
end

% Q & R were manually tuned to obtain a desired state response
Q = [20 0 0 0 0 0;
     0 20 0 0 0 0;
     0 0 1500 0 0 0;
     0 0 0 1500 0 0;
     0 0 0 0 1500 0;
     0 0 0 0 0 1500];

 R = 0.05;
 
 % Use LQR Controller to find a gain matrix to stabilize the system
 K = lqr(A, B, Q, R);
 
 % Construct open loop state-space representation
 SS_open = ss(A,B,C,D);
 
 % Construct closed loop state-space representation 
 A_closed = (A - B*K);
 SS_closed = ss(A_closed, B, C, D);
 
 % Define an arbitrary initial state
 X_0 = [0;
        0;
        12*pi/180;
        0;
        10*pi/180;
        0];
 t = 0:0.01:100;
 
 % Simulate open loop state space response to initial state
 figure
 initial(SS_open, X_0)
 grid on
 
 % Simulate closed loop state space response to initial state
 figure
 initial(SS_closed, X_0)
 grid on
 
 % Lyapunov Indirect method stability
 disp("Eigen Values: ")
 eig(A_closed)