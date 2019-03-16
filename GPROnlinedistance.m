clear;
clc;
close all


%%
% GRP code
% By yuxi

%% load the reference trajectory

InterpolationData=load('InterpInfo_Zhao_Fan.mat');
%InterpolationData=load('InterpInfo_Zhao_Blade.mat');
ToolTipPos=InterpolationData.interpcor(:,1:3);

x1 = ToolTipPos(:,1);                                                      % Set the parameters for C-MEX S function for hunting the foot point
y1 = ToolTipPos(:,2);
z1 = ToolTipPos(:,3);
U=InterpolationData.interpcor(:,9);

% Indentified the orientation vectors
lenOrienVector=length(U);
ToolOrienPos=zeros(lenOrienVector,3);
for ii=1:lenOrienVector
    TempOrienVector=InterpolationData.interpcor(ii,4:6);
    ToolOrienPos(ii,1:3)=InterpolationData.interpcor(ii,4:6)/norm(TempOrienVector);
end

i1 = ToolOrienPos(:,1);
j1 = ToolOrienPos(:,2);
k1 = ToolOrienPos(:,3);

DriveCommands = InverseKinematics_DH(ToolTipPos,ToolOrienPos);
HomePosDrive = DriveCommands(1,:);                                         % Set the home positon for motion control

%%
% load the actual trajectory

ActualPosition=load('ActualPosition.mat');
Trajectory = load('Trajectory_Zhao_Fan.mat');
%Trajectory = load('Trajectory_Zhao_Blade.mat');

[M,N] = size(ActualPosition.ActualPos);
ActualPositionData = zeros(M-1,N);
for ss=1:N
    ActualPositionData(1,ss) = ActualPosition.ActualPos(2,ss)+HomePosDrive(1);
    ActualPositionData(2,ss) = ActualPosition.ActualPos(3,ss)+HomePosDrive(2);
    ActualPositionData(3,ss) = ActualPosition.ActualPos(4,ss)+HomePosDrive(3);
    ActualPositionData(4,ss) = ActualPosition.ActualPos(5,ss)+HomePosDrive(4);
    ActualPositionData(5,ss) = ActualPosition.ActualPos(6,ss)+HomePosDrive(5);
end

[Pa,Oa] = ForwardKinematics_DH(ActualPositionData');
Pr=ToolTipPos;
[TrueTipContourError,TrueOrienContourError]=TipAndOrienConErrorCal(Trajectory,Pa',Pr,Oa);

x2 = Pa(:,1);
y2 = Pa(:,2);
z2 = Pa(:,3);
i2 = Oa(:,1);
j2 = Oa(:,2);
k2 = Oa(:,3);

%% use GPR mode to predict new distance and orientation X=[p q] F=[D Q]

k = @(x, y)exp(-500*(norm(x-y)).^2);            % 高斯核函数，该系数可以更改     253
N = size(x2); 
n = 3;                                         % n的变化对整体的预测效果影响不大
i = 1;        
xx = [InterpolationData.interpcor(i:n,1:3) ToolOrienPos(i:n,1:3)];         % reference      
yy = [Pa(i:n,1:3) Oa(i:n,1:3)];                                            % actual

%D_initial = norm(yy(:,1:3)-xx(:,1:3));
%D_initial = sqrt((yy(:,1)-xx(:,1)).^2 + (yy(:,2)-xx(:,2)).^2 +(yy(:,3)-xx(:,3)).^2);
%Ox_initial = (yy(:,1)-xx(:,1))./D_initial;
%Oy_initial = (yy(:,2)-xx(:,2))./D_initial;
%Oz_initial = (yy(:,3)-xx(:,3))./D_initial;
%D = D_initial;
%Ox = Ox_initial;
%Oy = Oy_initial;
%Oz = Oz_initial;

D_initial = zeros(n,1);
Ox_initial = zeros(n,1);
Oy_initial = zeros(n,1);
Oz_initial = zeros(n,1); 
O_initial = zeros(n,3);
for j = 1:n
    D_initial(j) = norm(yy(j,1:3)-xx(j,1:3));
    Ox_initial(j) = (yy(j,1)-xx(j,1))./D_initial(j);
    Oy_initial(j) = (yy(j,2)-xx(j,2))./D_initial(j);
    Oz_initial(j) = (yy(j,3)-xx(j,3))./D_initial(j);
end
D = D_initial;
Ox = Ox_initial;
Oy = Oy_initial;
Oz = Oz_initial;

for ii = n:N-1
    yy = [Pa(ii-n+1:ii,1:3) Oa(ii-n+1:ii,1:3)]; 
    yy_star = [Pa(ii+1,1:3) Oa(ii+1,1:3)];
    YY = [yy; yy_star];
    [M_train,~] = size(YY);
    for jj = 1:M_train
        for kk = 1:M_train
            KK_star(jj, kk) = k(YY(jj,:),YY(kk,:));
        end
    end
    K = KK_star(1:M_train-1, 1:M_train-1);
    K_star = KK_star(1:M_train-1, M_train);
    d_new(ii+1,:) = (K_star)' * pinv(K) * D;
    ox_new(ii+1,:) = (K_star)' * pinv(K) * Ox;
    oy_new(ii+1,:) = (K_star)' * pinv(K) * Oy;
    oz_new(ii+1,:) = (K_star)' * pinv(K) * Oz;
   
    % update the training data
    xx_next = [InterpolationData.interpcor(ii+1,1:3) ToolOrienPos(ii+1,1:3)];   
    yy_next = [Pa(ii+1,1:3) Oa(ii+1,1:3)];  
    
    %d_next = sqrt((yy_next(:,1)-xx_next(:,1)).^2 + (yy_next(:,2)-xx_next(:,2)).^2 +(yy_next(:,3)-xx_next(:,3)).^2);
    %ox_next = (yy_next(:,1)-xx_next(:,1))./d_next;
    %oy_next = (yy_next(:,2)-xx_next(:,2))./d_next;
    %oz_next = (yy_next(:,3)-xx_next(:,3))./d_next;
    
    d_next = norm(yy_next(:,1:3)-xx_next(:,1:3));
    ox_next = (yy_next(:,1)-xx_next(:,1))./d_next;
    oy_next = (yy_next(:,2)-xx_next(:,2))./d_next;
    oz_next = (yy_next(:,3)-xx_next(:,3))./d_next;
    
    D = [D(i+1:n); d_next];
    Ox = [Ox(i+1:n); ox_next];
    Oy = [Oy(i+1:n); oy_next];
    Oz = [Oz(i+1:n); oz_next];
end

GPR_D = [D_initial;d_new(n+1:N)]; 
GPR_Ox = [Ox_initial;ox_new(n+1:N)];
GPR_Oy = [Oy_initial;oy_new(n+1:N)];
GPR_Oz = [Oz_initial;oz_new(n+1:N)];
GPR_O = sqrt(GPR_Ox(:,:).^2 + GPR_Oy(:,:).^2 + GPR_Oz(:,:).^2);

%% plot
figure(1);
plot(TrueTipContourError(:,4),'b','Linewidth',2);
hold on;
plot(GPR_D,'r','Linewidth',2);
title('TipContourError');
legend('Ture','Learning');
hold off;

figure(2);
plot(TrueOrienContourError,'b','Linewidth',2);
hold on;
plot(GPR_O,'r','Linewidth',2);
title('TrueOrienContourError');
legend('Ture','Learning');
hold off;

figure(3);
tiperror = GPR_D - TrueTipContourError(:,4);
plot(tiperror,'b','Linewidth',2);
title('Tiperror');

figure(4);
Orienerror = GPR_O - TrueOrienContourError;
plot(Orienerror,'b','Linewidth',2);
title('Orienerror');

%% Plot the true contour error
%ProposedMethod=load('ConErrorProposed.mat');
%YangMethod=load('ConErrorYang.mat');
%CotourErrorPlot(TrueTipContourError,TrueOrienContourError,ProposedMethod.Proposed,YangMethod.Yang);



        











