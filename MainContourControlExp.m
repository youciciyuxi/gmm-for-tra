clc
clear
close all
%% Set general condition 
Ts=0.001;                                                           % Sample Period: 1ms 
VlimitX = 6; VlimitY = 6; VlimitZ = 6; VlimitA = 5; VlimitC = 5;    % Drives' input saturation value

%% Load trajectory data and Motion commands for single drives 
InterpolationData=load('InterpInfo_Zhao_Fan.mat');
% InterpolationData=load('InterpInfo_Zhao_Blade.mat');
ToolTipPos=InterpolationData.interpcor(:,1:3);

X=ToolTipPos(:,1);                                                      % Set the parameters for C-MEX S function for hunting the foot point
Y=ToolTipPos(:,2);
Z=ToolTipPos(:,3);
U=InterpolationData.interpcor(:,9);

% Indentified the orientation vectors
lenOrienVector=length(U);
ToolOrienPos=zeros(lenOrienVector,3);
for ii=1:lenOrienVector
    TempOrienVector=InterpolationData.interpcor(ii,4:6);
    ToolOrienPos(ii,1:3)=InterpolationData.interpcor(ii,4:6)/norm(TempOrienVector);
end

I=ToolOrienPos(:,1);
J=ToolOrienPos(:,2);
K=ToolOrienPos(:,3);


%% Calculate the Curvature
len=length(U);
DeBoorPointDer012=zeros(3,3,len);
for jj=1:len
    DeBoorPointDer012(:,:,jj)=DeBoorCoxNurbsCal(U(jj));
end
Curvature=CurvatureCal(DeBoorPointDer012);

DriveCommands =InverseKinematics_DH(ToolTipPos,ToolOrienPos);
ReferenceTrajPlot(ToolTipPos,ToolOrienPos,DriveCommands,Curvature,Ts);

RefPoints=DriveCommands;

HomePosDrive=DriveCommands(1,:);                                        % Set the home positon for motion control
Xcommands = DriveCommands(:,1)-HomePosDrive(1);            
Ycommands = DriveCommands(:,2)-HomePosDrive(2);          
Zcommands = DriveCommands(:,3)-HomePosDrive(3);  
Acommands = DriveCommands(:,4)-HomePosDrive(4);  
Ccommands = DriveCommands(:,5)-HomePosDrive(5);                         % To make sure the trajectory is from 0 position

%% Prepare Jacobin matrix for yang's method
[Jp,Jo] = JacobinFunction(DriveCommands);             % Jacobin Function
N=size(DriveCommands,1);
for kk=1:N
    CheckJp(3*(kk-1)+1:3*kk,:) = Jp(:,:,kk);
end

%% Set the plant X,Y,Z,A,Z parameters
[PlantXPara,PlantYPara,PlantZPara,PlantAPara,PlantCPara]=PlantParameters(Ts);
PlantXNum=[PlantXPara.b1 PlantXPara.b0];
PlantXDen=[1 PlantXPara.a1 PlantXPara.a0];
PlantYNum=[PlantYPara.b1 PlantYPara.b0];
PlantYDen=[1 PlantYPara.a1 PlantYPara.a0];
PlantZNum=[PlantZPara.b1 PlantZPara.b0];
PlantZDen=[1 PlantZPara.a1 PlantZPara.a0];
PlantANum=[PlantAPara.b1 PlantAPara.b0];
PlantADen=[1 PlantAPara.a1 PlantAPara.a0];
PlantCNum=[PlantCPara.b1 PlantCPara.b0];
PlantCDen=[1 PlantCPara.a1 PlantCPara.a0];

%% Simulate the model
sim('BSCForFiveAxisExperiment');

%% Calculate true tool tip contour error and tool orientation contour error
ActualPosition=load('ActualPosition.mat');
Trajectory = load('Trajectory_Zhao_Fan.mat');
% Trajectory = load('Trajectory_Zhao_Blade.mat');
ProposedMethod=load('ConErrorProposed_new.mat');
YangMethod=load('ConErrorYang.mat');
[M,N]=size(ActualPosition.ActualPos);
ActualPositionData=zeros(M-1,N);
for ss=1:N
    ActualPositionData(1,ss)=ActualPosition.ActualPos(2,ss)+HomePosDrive(1);
    ActualPositionData(2,ss)=ActualPosition.ActualPos(3,ss)+HomePosDrive(2);
    ActualPositionData(3,ss)=ActualPosition.ActualPos(4,ss)+HomePosDrive(3);
    ActualPositionData(4,ss)=ActualPosition.ActualPos(5,ss)+HomePosDrive(4);
    ActualPositionData(5,ss)=ActualPosition.ActualPos(6,ss)+HomePosDrive(5);
end
[Pa,Oa] = ForwardKinematics_DH(ActualPositionData');
Pr=ToolTipPos;
[TrueTipContourError,TrueOrienContourError]=TipAndOrienConErrorCal(Trajectory,Pa',Pr,Oa);

%% Plot the estimated contour and the true contour error
CotourErrorPlot(TrueTipContourError,TrueOrienContourError,ProposedMethod.Proposed,YangMethod.Yang);









