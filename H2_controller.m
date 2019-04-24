% This function calculates the optimal H_2 robust controller for and LQG 
% application
%% INITIALIZATION
clc; clear all; close all;

% Intro
disp('H_2 - LQG control on discrete time system');
disp('--------------------------------------');
%disp('  Simulated Experiments  ');
fprintf('\n');

%disp('Load customized paths from Yalmip Toolbox');
%run('load_paths');

% %  Load Paths
% disp('Load customized paths of PRtools and DIPimage etc');
% run('load_paths');
%%
% Initial matrices
a = [1.01 0.01 0]
A_true = toeplitz(a,a')
B_true = eye(3);

C = eye(3);
Q = 10^(2   ) * eye(3);
R = eye(3);
W = eye(3);
V = eye(3);

% Initialization of matrices
A = A_true;
B_1 = sqrt(W); B_2 = B_true;
C_1 = sqrt(Q); C_2 = C;
D_11 = zeros(size(B_1)); D_22 = zeros(size(B_2));
D_12 = sqrt(R); D_21 = sqrt(V);

% Matrices useful for finding controller.
A_x = A - B_2 * D_12' * C_1; 
A_y = A - B_1 * D_21' * C_2;
R = eye(length(A_x)); E = eye(length(A_x));

[X_2,K_1,L] = dare(A_x,B_2,zeros(size(A_x)),R,[],E);

[Y_2,K_2,L] = dare(A_y,C_2,zeros(size(A_x)),R,[],E)
R_b = eye(length(B_2)) + B_2' * X_2 * B_2;
F_2 = -inv(eye(length(B_2)) + B_2' * X_2 * B_2) * (B_2' * X_2 * A + D_12' * C_1);
F_0 = -inv(eye(length(B_2)) + B_2' * X_2 * B_2) * (B_2' * X_2 * B_1 +D_12' * D_11);
L_2 = -(A * Y_2 *C_2' + B_1 * D_21') * inv(eye(length(C_2')) + C_2 * Y_2 * C_2');
L_0 = (F_2 * Y_2 * C_2' + F_0 * D_21') * inv(eye(length(C_2')) + C_2 * Y_2 * C_2')

A_F2 = A + B_2 * F_2;
A_L2 = A + L_2 * C_2;
C_1F2 = C_1 + D_12 * F_2;
B_1L2 = B_1 + L_2 * D_21;
A_hat_2 = A + B_2 * F_2 + L_2 * C_2;

A_K = A_hat_2-B_2*L_0*C_2;
B_K = -(L_2 -  B_2*L_0);
C_K = (F_2 - L_0*C_2);
D_K = L_0;
    
% Preparation of the simulation.
steps = 50;
x = zeros(3,steps);
xi = zeros(3,steps);
x_opt = zeros(3,steps);

N = zeros(3,3);
[K_lqr,S,e] = dlqr(A_true,B_true,Q,R,N)


%%
J_K = 0;
J_opt = 0;
for j=1:steps
    w = randn(3,1);  
    
    x_opt(:,j+1) = (A_true - B_true * K_lqr) * x_opt(:,j) + w;
    y(:,j) = C_2 * x(:,j) + D_21 * w;
    % Controller dynamics
    u(:,j) = C_K * xi(:,j) + D_K * y(:,j);
    xi(:,j+1) = A_K * xi(:,j) + B_K * y(:,j);
    
    x(:,j+1) = A * x(:,j) + B_2 * u(:,j) + B_1 * w;
    z(:,j) = C_1 * x(:,j) + D_11 * w + D_12 * u(:,j);
    
    J_opt = J_opt + x_opt(:,j+1)'*(Q + K_lqr' * R * K_lqr)*x_opt(:,j+1);
    J_K = J_K + x(:,j+1)'* Q *x(:,j+1) + u(:,j)' * R * u(:,j);

end

cost_opt = J_opt
cost_K = J_K