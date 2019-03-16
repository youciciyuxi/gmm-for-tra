%%
%GRP code
%Author Liang
%%
%data produce
xx=0:0.1:5;
yy=5*rand(1,51);
xxx=0:0.1:10;
zzz=sin(xx)+yy;
plot3(xx,yy,zzz,'g+');
hold on
X=[xx' yy'];
X_star=[2.5,1.5];
X=[X;X_star];
%%
%use GPR mode to predict dist;
[N_train,~]=size(X)
k=@(x,y)exp(-2*norm(x-y)^2);
for ii = 1:N_train
    for j = 1:N_train
        K0(ii,j) = k(X(ii,:),X(j,:));
    end
end
K=K0(1:N_train-1,1:N_train-1);
K_star=K0(1:N_train-1,N_train);
%%
%caculate the result and plot it

Y_new=(K_star')*inv(K)*(zzz');
plot3(X_star(1),X_star(2),Y_new,'b*');
hold on;




