%% INITIALIZATION
clc; clear all; close all;

% Intro
disp('Safe Reinforcement Learning');
disp('--------------------------------------');
disp('  Experimenting with Yalmip  ');
fprintf('\n');

disp('Load customized paths from Yalmip Toolbox');
run('load_paths');
%% Supposing matrices
at = [1.01 0.01 0]
A = toeplitz(at,at')
B = eye(3);

%% Getting Started: Defining the opt. problem.
Q = 10^(-3) * eye(3);
R = eye(3);
G = chol(eye(3));

% Variables
Z = sdpvar(3,3);
Y = sdpvar(3,3);
L = sdpvar(3,3);

% Constraints
C = [[Z G;G' Y]>=0, [Y Y*A'+L'*B' Y * sqrt(Q) L'; A*Y+B*L Y zeros(3,3) zeros(3,3); sqrt(Q)*Y zeros(3,3) eye(3) zeros(3,3); L zeros(3,3) zeros(3,3) inv(R)]>=0];
objective = trace(Z);
%% Optimization
sol = optimize(C,objective)
K = value(L) * inv(value(Y));
A_new = A + B*K