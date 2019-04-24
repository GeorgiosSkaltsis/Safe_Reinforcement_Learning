%% LQR cost of matrix K

function [cost, cost_opt, ratio] = lqr_cost(K)
steps = 50
System's matrices
a = [1.01 0.01 0]
A_true = toeplitz(a,a')
B_true = eye(3);

%Calculate the proper lqr matrix
Q = 10^(-3) * eye(3);
R = eye(3); N = zeros(3,3);
[K_lqr,S,e] = dlqr(A_true,B_true,Q,R,N)

x_K = zeros(3,steps);
x_opt = zeros(3,steps);
J_K = 0;
J_opt = 0;
for j=1:steps
    noise = randn(3,1);   
    x_opt(:,j+1) = (A_true - B_true * K_lqr) * x_opt(:,j) + noise;
    x_K(:,j+1) = (A_true + B_true * K) * x_K(:,j) + noise;
   
    J_opt = J_opt + x_opt(:,j+1)'*(Q + K_lqr' * R * K_lqr)*x_opt(:,j+1);
    J_K = J_K + x_K(:,j+1)'*(Q + K' * R * K)*x_K(:,j+1);

end
cost = J_K;
cost_opt = J_opt;
ratio = cost / cost_opt;

end


