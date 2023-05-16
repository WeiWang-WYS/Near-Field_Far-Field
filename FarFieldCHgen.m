function  [H_matrix Psi_BS Varphi_BS Psi_UAV Varphi_UAV]= UnifiedCHgen(AntDimBS_1, AntDimBS_2, AntDimUAV_1, AntDimUAV_2, Pos_BS, Pos_UAV, Lambda, RotMatBS, RotMatUAV)

SteeringVector=@(theta, AntNum) exp(j*pi*(0:1:AntNum-1)'*theta);


%%
dis_UAV2BS = norm(Pos_UAV - Pos_BS,2); %%% Distance between UAV and BS
Gain = Lambda/(4*pi*dis_UAV2BS)*exp(-j*2*pi*dis_UAV2BS/Lambda); %%% Free-space path loss

%% Angle in BS side
DirUAV2BS = (Pos_BS - Pos_UAV)/dis_UAV2BS;  %% Direction vector 
VecTemp = RotMatBS.'*DirUAV2BS; %% Inner product
Psi_BS = VecTemp(1); %% Angles
Varphi_BS = VecTemp(3);


%% Angle in UAV side
VecTemp = RotMatUAV.'*DirUAV2BS; %% Inner product
Psi_UAV = VecTemp(1);  %% Angles
Varphi_UAV = VecTemp(2);

%% Steering vectors
v_BS = kron(SteeringVector(Psi_BS, AntDimBS_1), SteeringVector(Varphi_BS, AntDimBS_2)); 
v_UAV = kron(exp(-j*(AntDimUAV_1-1)/2*pi*Psi_UAV)*SteeringVector(Psi_UAV, AntDimUAV_1), exp(-j*(AntDimUAV_2-1)/2*pi*Varphi_UAV)*SteeringVector(Varphi_UAV, AntDimUAV_2));
H_matrix = Gain*v_UAV*v_BS';
