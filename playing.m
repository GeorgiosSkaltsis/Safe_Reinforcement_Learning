clc; clear all; close all;

% Intro
disp('Playing with matrices');
disp('--------------------------------------');
fprintf('\n');
A = [0.67 0.67 0 0; -0.67 0.67 0 0; 0 0 -0.67 -0.67; 0 0 0.67 -0.67]
answer = A^100

% a = [1.01 0.01 0]
% A_true = toeplitz(a,a')
% pl = A_true^100
