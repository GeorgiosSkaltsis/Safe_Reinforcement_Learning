%% INITIALIZATION
clc; clear all; close all;

% Intro
disp('Safe Reinforcement Learning');
disp('--------------------------------------');
disp('  Simulated Experiments  ');
fprintf('\n');

% %  Load Paths
% disp('Load customized paths of PRtools and DIPimage etc');
% run('load_paths');
%% Initialize Matrices
a = [1.01 0.01 0]
A_true = toeplitz(a,a')
