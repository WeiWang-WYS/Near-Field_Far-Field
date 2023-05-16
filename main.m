% ---------------------------------------------------------------------------------------------------------------------------------
%  (1) The repository provides two ways of channel modelling, i.e., the unified channel model (the elemment of which is determined by the BS-UAV antenna pair distance) and the far-field channel model (which is parameterized by AoA/AoD), 
%      to illustrate the difference between far-field and near-field wireless channels.
%  (2) The code can be used in simulations for the MIMO communication link with arbitary Tx/Rx array shapes and orientations.
%  (3) For details of the code, please refer to ¡°Jittering effects analysis and beam training design for UAV millimeter wave communications,¡± IEEE Transactions on Wireless Communications, vol. 21, no. 5, pp. 3131-3146, May 2022.
% ---------------------------------------------------------------------------------------------------------------------------------

clc;
clear;

%% Parameters
Pos_UAV_Ini = [0, .5, 0]'; %% The initial position of UAV
Pos_BS = [0, 0, 0]';  %% The position of BS
Velocity = 1; %% The velocity of UAV
Vel_dir = [-1, 0, 0]'; %% The direction of UAV movement
LightSpeed = 3*10^8; %% The speed of light
Frequency = 28*10^9; %% The operating frequency: 28GHz
AntBS_Dim_1 = 16;  %% Size of the UPA in BS
AntBS_Dim_2 = 16;
AntUAV_Dim_1 = 8; %% Size of the UPA in UAV
AntUAV_Dim_2 = 4;
Lambda = LightSpeed/Frequency; %% Wavelength
time_seq = 0:1:1000; %%
SteeringVector=@(theta, AntNum) exp(j*pi*(0:1:AntNum-1)'*theta); %% ULA Steering Vector

%% Configuration of UPA in BS & UAV side
for ix = 1:1:AntBS_Dim_1
    for iz = 1:1:AntBS_Dim_2
        BS_Ant_RLPos(:, (ix-1)*AntBS_Dim_2 + iz) = Lambda/2*[ix-1, 0, iz-1]';  %%% Relative location of the antenna elements in the BS side (along x axis and z axis)  
    end
end
for ix = 1:1:AntUAV_Dim_1
    for iy = 1:1:AntUAV_Dim_2
        UAV_Ant_RLPos(:, (ix-1)*AntUAV_Dim_2 + iy) = Lambda/2*[ix-1-(AntUAV_Dim_1-1)/2, iy-1-(AntUAV_Dim_2-1)/2, 0]'; %%%% Relative location of the antenna elements in the UAV side (along x axis and y axis)  
    end
end
%-- You can set an arbitary array shape by customizing your own BS_Ant_RLPos and UAV_Ant_RLPos


figure(1)
subplot(2,1,1)
plot3(BS_Ant_RLPos(1,:), BS_Ant_RLPos(2,:), BS_Ant_RLPos(3,:), '.')
xlabel('x'),ylabel('y'),zlabel('z');
title('UPA in BS side');
subplot(2,1,2)
plot3(UAV_Ant_RLPos(1,:), UAV_Ant_RLPos(2,:), UAV_Ant_RLPos(3,:), '.')
xlabel('x'),ylabel('y'),zlabel('z');
title('UPA in UAV side');

%% Attitude/Orientation angles of the UAV
alpha_ini = 1; %% Yaw
beta_ini = 0; %% Pitch
gamma_ini = 0; %% Roll

%% Attitude/Orientation angles of the BS
RotMatBS = eye(3); 

%%
for iT = 1:1:length(time_seq)
%% Jittering/Wobbling effects of the mobile terminal. 
    alpha = alpha_ini + 0.03*randn(1);
	beta = beta_ini + 0.03*randn(1);
	gamma = gamma_ini + 0.03*randn(1);
    RotYaw = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
    RotPitch = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
    RotRoll = [1 0 0; 0 cos(gamma) -sin(gamma); 0 sin(gamma) cos(gamma)];
	RotMatUAV = RotYaw*RotPitch*RotRoll;
%% Position of UAV at the time interval iT  
  	Pos_UAV = Pos_UAV_Ini + Vel_dir*Velocity*time_seq(iT); 
%% The unified far-field/near-field channel model,and the elemment of which is determined by the BS-UAV antenna pair distance.
    H_matrix_unified = UnifiedCHgen(BS_Ant_RLPos, UAV_Ant_RLPos, Pos_BS, Pos_UAV, Lambda,RotMatBS, RotMatUAV);  
%% The far-field channel model£¬which is parameterized by AoA/AoD.
    [H_matrix_far_field Psi_BS Varphi_BS Psi_UAV Varphi_UAV]= FarFieldCHgen(AntBS_Dim_1, AntBS_Dim_2, AntUAV_Dim_1, AntUAV_Dim_2, Pos_BS, Pos_UAV, Lambda, RotMatBS, RotMatUAV);
%% NMSE between the unified channel model and the far-field channel model.
    NMSE_seq(iT) = norm(H_matrix_unified - H_matrix_far_field,2)^2/norm(H_matrix_unified,2)^2;   %% The mean squared error between the unified far-field/near-field channel model and the simplified far-field channel model that is parameterized by AoA/AoD. 
    dis_seq(iT) = norm(Pos_UAV-Pos_BS,2);
end   

figure(2)
semilogy (dis_seq, NMSE_seq, '-')
xlabel('Distance/meter')
ylabel('NMSE')

