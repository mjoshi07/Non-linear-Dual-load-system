% Define Initial State
X_0 = [15; 0; 12; 0; 10; 0];
t = 0:0.01:2000;

% Use MATLAB's Non Linear solver
[t1,x] = ode45(@non_linear_,t,X_0);
subplot(2,3,1);
plot(t1,x(:,1));
title("State: x")
grid on
subplot(2,3,2);
plot(t1, x(:,2));
title("State: x-dot")
grid on
subplot(2,3,3);
plot(t1, x(:, 3));
title("State: theta-1")
grid on
subplot(2,3,4);
plot(t1, x(:,4));
title("State: theta-1-dot")
grid on
subplot(2,3,5);
plot(t1, x(:,5));
title("State: theta-2")
grid on
subplot(2,3,6);
plot(t1, x(:,6));
title("State: theta-2-dot")
grid on

function X_dot = non_linear_(t, x)
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
    
    Q = [20 0 0 0 0 0;
     0 20 0 0 0 0;
     0 0 1500 0 0 0;
     0 0 0 1500 0 0;
     0 0 0 0 1500 0;
     0 0 0 0 0 1500];

    R = 0.05;
    
    % Design LQR control
    K = lqr(A, B, Q, R);
    F = -K*x;
    
    % Compute State space transition
    denominator = M+m1*((sind(x(3)))^2)+m2*((sind(x(5)))^2);
    X_double_dot = (F-(g/2)*(m1*sind(2*x(3))+m2*sind(2*x(5)))-(m1*l1*(x(4)^2)*sind(x(3)))-(m2*l2*(x(6)^2)*sind(x(5))))/(denominator);
    theta1_double_dot = (X_double_dot*cosd(x(3))-g*(sind(x(3))))/l1;
    theta2_double_dot = (X_double_dot*cosd(x(5))-g*(sind(x(5))))/l2;
    X_dot = zeros(6,1);
    X_dot(1) = x(2);
    X_dot(2) = X_double_dot;
    X_dot(3) = x(4);
    X_dot(4) = theta1_double_dot;
    X_dot(5) = x(6);
    X_dot(6) = theta2_double_dot;
end


    