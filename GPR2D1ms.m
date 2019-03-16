%%
%GPR code
%By yuxi

%%  load the date of trajectory
% Butterfly
load('Butterfly.mat');        % Desired trajectory   
x = Xcommand;
y = Ycommand;
plot(x, y, 'g-', 'Linewidth',2);   
hold on;

load('exp00.mat');                     % Actual trajectory            
k = @(x, y)exp(-5*(norm(x-y))^2);      %coefficient is less is better

%N = size(Xcommand)-1; 
N = 3000;
n = 3;                                % Set the orignal data
for i = 1:N-n
    xx = Xcommand(i:n+i);
    yy = Ycommand(i:n+i);
    plot(xx, yy, 'r--', 'Linewidth',2);
    hold on;
    
    X_star = Xcommand(n+i+1);
    X = [xx; X_star];
    
    % use GPR mode to predict new data
    [M_train, ~] = size(X);
    for ii = 1:M_train
        for jj = 1:M_train
            KK_star(ii, jj) = k(X(ii), X(jj));
        end
    end
    K = KK_star(1:M_train-1, 1:M_train-1);
    K_star = KK_star(1:M_train-1, M_train);
    
    % calculate the Normal distribution of K_star 
    New_star = (K_star)' * pinv(K) * yy;
    plot(X_star, New_star, 'b*','Linewidth',0.001);
    hold on;
end
legend('Desired', 'PID Actual', 'GPR');
%legend('Desired', 'Desired', 'GPR');
title('Trajectory');
            











