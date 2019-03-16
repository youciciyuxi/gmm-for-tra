clc
clear
close all
%% Set general condition 
Ts=0.001;                                                           % Sample Period: 1ms 
VlimitX = 6; VlimitY = 6; VlimitZ = 6; VlimitA = 5; VlimitC = 5;    % Drives' input saturation value

%% Load trajectory data 
InterpolationData=load('InterpInfo_Zhao_Fan.mat');
%InterpolationData=load('InterpInfo_Zhao_Blade.mat');
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

DriveCommands =InverseKinematics_DH(ToolTipPos,ToolOrienPos);
HomePosDrive=DriveCommands(1,:);                                        % Set the home positon for motion control

%% Calculate true tool tip contour error and tool orientation contour error
ActualPosition=load('ActualPosition.mat');
Trajectory = load('Trajectory_Zhao_Fan.mat');
%Trajectory = load('Trajectory_Zhao_Blade.mat');

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
CotourErrorPlot(TrueTipContourError,TrueOrienContourError);









