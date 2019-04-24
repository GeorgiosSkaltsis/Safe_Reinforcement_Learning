%% INITIALIZATION
clc; clear all; close all;

% Intro
disp('Safe Reinforcement Learning');
disp('--------------------------------------');
disp('  Simulated Experiments  ');
fprintf('\n');

disp('Load customized paths from Yalmip Toolbox');
run('load_paths');

% %  Load Paths
% disp('Load customized paths of PRtools and DIPimage etc');
% run('load_paths');
%% Initialize Matrices
disp('--------------------------------------');
disp('  Initializing matrices  ');
fprintf('\n');

a = [1.01 0.01 0]
A_true = toeplitz(a,a')
B_true = eye(3);


%% Performing the N roll-outs: Recht's Least Squares approach.


disp('--------------------------------------');
disp('  Creating the roll - outs.  ');
fprintf('\n');
% Number of roll - outs
N = 60;
steps = 10;
x = zeros(3,N,steps);
u = zeros(3,N,steps);
Z_N = [];
X_N = [];

for i=1:N
    for j=1:steps
        % I should save the inputs as well. I will need them!
    u(:,i,j) = randn(3,1);
    noise = randn(3,1);
    Z_N = [Z_N; x(:,i,j)' u(:,i,j)'];    
    
    x(:,i,j+1) = A_true * x(:,i,j)+ B_true * u(:,i,j) + noise;
    X_N = [X_N; x(:,i,j+1)'];
    end
end
Theta = inv(Z_N' * Z_N) * Z_N' * X_N;
disp('--------------------------------------');
disp('  And the mean matrices are:  ');
fprintf('\n');

B = Theta(4:6,1:3)'
A = Theta(1:3,1:3)'
updated = 0;
if updated == 1
    disp('--------------------------------------');
    disp('  Running for the updated scenario');
    fprintf('\n');
    A = A_true;
    B = B_true;
end
C = eye(3);
Q = 10^(-3) * eye(3);
R = eye(3);
W = eye(3);
V = eye(3);
% [K] = H2_controller(A,B,C,Q,R,W,V)


%% On calculating the likelihood distribution
% Sum = 0;
% Sum_2 = 0;
% for i=1:N
%   for j=1:steps
%     Mia = [x(:,i,j)' u(:,i,j)'];
%     D = kron(eye(3),Mia);
%     Sum = Sum + D' * D; %Pi is the variance of the noise
%     Sum_2 = Sum_2 + D' * x(:,i,j+1)
%   end
% end
% Sigma = inv(Sum)
% mu = Sigma * Sum_2;
%%
% A_exp = [mu(1:3)'; mu(7:9)'; mu(13:15)']
% B_exp = [mu(4:6)'; mu(10:12)'; mu(16:18)']
% % Constructing the prior distribution with mean on LS and identical
% % variance
% L = chol(Sigma);
% M = 5
% for i=2:M
%    mu_temp = L * randn(18,1) + mu;
%    A_exp(:,:,i) = [mu_temp(1:3)'; mu_temp(7:9)'; mu_temp(13:15)']
%    B_exp(:,:,i) = [mu_temp(4:6)'; mu_temp(10:12)'; mu_temp(16:18)']
%    U_C = [B_exp(:,:,i) A_exp(:,:,i)*B_exp(:,:,i) (A_exp(:,:,i))^2*B_exp(:,:,i)];
%    [h,p,ci,stats] = vartest(mu_temp,Sigma) 
%    rankC = rank(U_C)
% end
% 
% 
%% Creating pairs of matrices A and B: In favor of Bayesian inference.
% % The purpose here is to find what kind of noise should we add on the two
% % matrices!
% disp('--------------------------------------');
% disp('  Matrices inference:  ');
% fprintf('\n');
% M = 10; % Number of Monte Carlo approximations
% for i=2:M
%    A(:,:,i) = A(:,:,1) + 0.1 * randn(3,3);
%    B(:,:,i) = B(:,:,1) + 0.1 * randn(3,3);
%    U_C = [B(:,:,i) A(:,:,i)*B(:,:,i) A(:,:,i)^2*B(:,:,i)];
%    rankC = rank(U_C);
% end
% 
%% Solving an LMI for Common Lyapunov Relaxation
% disp('--------------------------------------');
% disp('  Preparing the LMI:  ');
% fprintf('\n');
% 
% Q = 10^(-3) * eye(3);
% R = eye(3);
% G = chol(eye(3));
% 
% % Variables
% % Z = sdpvar(3,3,M);
% Y = sdpvar(3,3);
% L = sdpvar(3,3);
% 
% % Constraints
% C = []
% for i=1:M
%   Z{i} = sdpvar(3,3);
%   AA = A(:,:,i);
%   BB = B(:,:,i);
%   Con = [[Z{i} G;G' Y]>=0, [Y Y*AA'+L'*BB' Y * sqrt(Q) L'; AA*Y+BB*L Y zeros(3,3) zeros(3,3); sqrt(Q)*Y zeros(3,3) eye(3) zeros(3,3); L zeros(3,3) zeros(3,3) inv(R)]>=0];
%   C = [C + Con]  
% end
% 
% % C = [[Z G;G' Y]>=0, [Y Y*A'+L'*B' Y * sqrt(Q) L'; A*Y+B*L Y zeros(3,3) zeros(3,3); sqrt(Q)*Y zeros(3,3) eye(3) zeros(3,3); L zeros(3,3) zeros(3,3) inv(R)]>=0];
% obj = 0;
% for i=1:M
%     obj = obj + trace(Z{i});
% end
% objective = obj;
%% Optimization
% disp('--------------------------------------');
% disp('  Solving the LMI:  ');
% fprintf('\n');
% 
% sol = optimize(C,objective)
% K = value(L) * inv(value(Y));
% % A_new = A + B*K
% for i=1:M
%     A_new = A(:,:,i)+B(:,:,i)*K;
%     eigA = eig(A_new);
%     X_bar(:,:,i) = inv(value(Y));
% end
% disp('  And the costs are:  ');
% fprintf('\n');
% [cost,cost_opt,ratio] = lqr_cost(value(K))
% 
% pause(1);
% 
% cost_previous = cost * 100; % just a big value instead of infinity!
% epsilon = 1;
% % X_bar(:,:,:) = inv(value(Y));
%% Iterative improvement by sequential SDP
% counter = 0;
% costs = [];
% while (abs(cost_previous - cost) >= epsilon )
%     disp('--------------------------------------');
%     disp('  Iterative Improvement:  ');
%     fprintf('\n');
% 
%     K_bar = K;
%     K = sdpvar(3,3);
% 
%     % Constraints
%     C2 = []
% 
%     for i=1:M
%       X{i} = sdpvar(3,3);
%       AA = A(:,:,i);
%       BB = B(:,:,i);
%       Con2 = [[X{i}-Q (AA+BB*K)' K'; AA+BB*K inv(X_bar(:,:,i))-inv(X_bar(:,:,i))*(X{i}-X_bar(:,:,i))*inv(X_bar(:,:,i)) zeros(3,3); K zeros(3,3) inv(R)]>=0];
%       C2 = [C2 + Con2]  
%     end
% 
% 
%     obj = 0;
%     for i=1:M
%         obj = obj + trace(X{i}); %Pi_i is identity.
%     end
%     objective = obj;
% 
%     % Optimization
%     disp('Iterative improvement by sequential SDP');
%     sol = optimize(C2,objective)
% 
%     for i=1:M
%       X_bar(:,:,i) = value(X{i})
%     end
% 
%     % Calculate lqr-cost
%     counter = counter + 1;
%     
%     cost_previous = cost;
%     [cost,cost_opt,ratio] = lqr_cost(value(K))
%     costs(counter) = cost;
%     disp('--------------------------------------');
%     disp('  Pause for 3 seconds  ');
%     fprintf('\n');
%     pause(1)
%     disp('--------------------------------------');
%     disp('  The new value of the Summation of Xis  ');
%     disp(value(objective));
%     fprintf('\n');
%     disp('--------------------------------------');
%     pause(2)
% end
% 
% disp('--------------------------------------');
% X = sprintf('  The iterative procedure took %d iterations.  ',counter);
% disp(X);
% fprintf('\n');
