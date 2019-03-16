%%
%GPR code
%By yuxi

%%  load the date of trajectory
% Butterfly
load('Butterfly.mat');        % Desired trajectory   
x = Xcommand;
y = Ycommand;
z = Zcommand;
plot3(x, y, z, 'b-');
hold on;

load('exp00.mat');             % Actual trajectory
xx = Xcommand(1:1000);
yy = Ycommand(1:1000);
zz = Zcommand(1:1000);
plot3(xx, yy, zz, 'r--');
view(0, 90);
hold on;

X = [xx yy zz];
X_star = [30 0 0];
X = [X; X_star];

%%
% use GPR mode to predict new data
[M_train, ~] = size(X);
k = @(x, y)(-5*norm(x-y)^2);
for ii = 1:M_train
    for jj = 1:M_train
        KK_star(ii, jj) = k(X(ii, : ), X(jj, : ));
    end
end
K = KK_star(1:M_train-1, 1:M_train-1);
K_star = KK_star(1:M_train-1, M_train);
        
%% calculate the Normal distribution of K_star 
New_star = (K_star)' * pinv(K) * [yy zz];
%cov_star = -(K_star)'*inv(K) * K_star + KK_star(M_train, M_train);
%F_star = 1/(sqrt(2*pi) * cov_star) * exp(-(X_star-mean_star)^2/(2*cov_star^2));
plot3(X_star(1), New_star(1), New_star(2), 'b*')










