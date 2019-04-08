
%% LMI solving: Minimize Linear Objectives under LMI Constraints.
%  a = [-1 -2 1;3 2 1;1 -2 -1]
%  b = [1;0;1]
%  q = [1 -1 0; -1 -3 -12; 0 -12 -36]
%  
%  setlmis([]) 
% X = lmivar(1,[3 1]) % variable X, full symmetric
% 
% lmiterm([1 1 1 X],1,a,'s') 
% lmiterm([1 1 1 0],q) 
% lmiterm([1 2 2 0],-1) 
% lmiterm([1 2 1 X],b',1)
% 
% LMIs = getlmis
% 
% c = mat2dec(LMIs,eye(3))
% options = [1e-5,0,0,0,0] 
% 
% [copt,xopt] = mincx(LMIs,c,options)
% Xopt = dec2mat(LMIs,xopt,X)

%% Performing the N roll-outs: A MOESP approach
% % % disp('--------------------------------------');
% % % disp('  Creating the roll - outs.  ');
% % % fprintf('\n');
% % % N = 60;
% % % steps = 6;
% % % x = zeros(3,N,steps);
% % % u = zeros(3,N,steps);
% % % U_k = [];
% % % X_k_1 = [];
% % % X_k = [];
% % % Y = [];
% % % U = [];
% % % noise_k = [];
% % % V = [];
% % % for i=1:N
% % %     for j=1:steps
% % %         % I should save the inputs as well. I will need them!
% % %     u(:,i,j) = randn(3,1);
% % %     noise = randn(3,1);
% % %     U_k = [U_k ; u(:,i,j)];
% % %     noise_k = [noise_k ; noise];
% % %     x(:,i,j+1) = A_true * x(:,i,j)+ B_true * u(:,i,j) + noise;
% % %     X_k_1 = [X_k_1 ; x(:,i,j+1)];
% % %     X_k = [X_k ; x(:,i,j)];
% % %     end
% % %     Y = [Y  X_k];
% % %     U = [U  U_k];
% % %     V = [V noise_k];
% % %     noise_k = [];
% % %     X_k = [];
% % %     U_k = [];
% % % end
% % % 
% % % % EXP = V * U';
% % % % Naive approach - or, noiseless approach
% % % Ts_hat = Y * U' * inv(U * U');
% % % B_ver = Ts_hat(4:6,1:3)
% % % A_ver = Ts_hat(7:9,1:3)/B_ver

% % 
% % Projection = eye(60) - U' * inv(U * U') * U;
% % ZERO = U * Projection;
% % % V_hat = Y * Projection * inv(Projection);
% % % Ts_hat_exp = (Y-V_hat) * U' * inv(U * U');
% % % B_exp = Ts_hat(4:6,1:3)
% % % A_exp = Ts_hat(7:9,1:3)/B_exp
% % % Error = V -V_hat;
% % 
% % c = eye(3);
% % d = zeros(3,3);

% % % % To compare with the actual system, we use the bode diagram:
% % %     w = [0:0.005:0.5]*(2*pi); 		% Frequency vector
% % %     m1 = dbode(A_true,B_true,c,d,1,1,w);
% % %     m2 = dbode(A_true,B_true,c,d,1,2,w);
% % %     m3 = dbode(A_true,B_true,c,d,1,3,w);
% % %     M1 = dbode(A,B,c,d,1,1,w);
% % %     M2 = dbode(A,B,c,d,1,2,w);
% % %     M3 = dbode(A,B,c,d,1,3,w);
% % % % 
% % % % Plot comparison
% % %     figure(1)
% % %     hold off;subplot;%clg;
% % %     subplot(331);plot(w/(2*pi),[m1(:,1),M1(:,1)]);title('Input 1 -> Output 1');
% % %     subplot(332);plot(w/(2*pi),[m2(:,1),M2(:,1)]);title('Input 2 -> Output 1');
% % %     subplot(333);plot(w/(2*pi),[m3(:,1),M3(:,1)]);title('Input 3 -> Output 1');
% % %     
% % %     subplot(334);plot(w/(2*pi),[m1(:,2),M1(:,2)]);title('Input 1 -> Output 2');
% % %     subplot(335);plot(w/(2*pi),[m2(:,2),M2(:,2)]);title('Input 2 -> Output 2');
% % %     subplot(336);plot(w/(2*pi),[m3(:,2),M3(:,2)]);title('Input 3 -> Output 2');
% % %     
% % %     subplot(337);plot(w/(2*pi),[m1(:,3),M1(:,3)]);title('Input 1 -> Output 3');
% % %     subplot(338);plot(w/(2*pi),[m2(:,3),M2(:,3)]);title('Input 1 -> Output 3');
% % %     subplot(339);plot(w/(2*pi),[m3(:,3),M3(:,3)]);title('Input 2 -> Output 3');
