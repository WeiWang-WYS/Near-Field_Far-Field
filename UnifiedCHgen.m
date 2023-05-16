function  H_matrix = UnifiedCHgen(BS_Ant_RLPos, UAV_Ant_RLPos, Pos_BS, Pos_UAV, Lambda, RotMatBS, RotMatUAV)


BS_Ant_Pos  = RotMatBS*BS_Ant_RLPos + Pos_BS*ones(1,size(BS_Ant_RLPos,2));  %%% Absolute position of the antenna elements of the BS 
UAV_Ant_Pos = RotMatUAV*UAV_Ant_RLPos + Pos_UAV*ones(1,size(UAV_Ant_RLPos,2)); %%% Absolute position of the antenna elements of the UAV 

for i_Ant_BS = 1:1:length(BS_Ant_Pos)
    Dis_Matrix(i_Ant_BS, :) = sqrt(sum((BS_Ant_Pos(:,i_Ant_BS)*ones(1, length(UAV_Ant_Pos)) - UAV_Ant_Pos).*(BS_Ant_Pos(:,i_Ant_BS)*ones(1, length(UAV_Ant_Pos)) - UAV_Ant_Pos))); %%% Distance of the BS-UAV antenna pairs
end

Dis_Matrix = Dis_Matrix.';
H_matrix = Lambda./(4*pi*Dis_Matrix).*exp(-j*2*pi*Dis_Matrix/Lambda);

%  ---------------------------------------------------------------------------------------------------------------------------------
%  According to the free-space path loss model in Eq. (2.7) and Eq. (2.8) of [1], the complex coefficient of the LoS path between the ¦Ê-th element antenna in UAV side and
%  the ¦É-th element antenna at BS side (the BS-UAV antenna pair) can be represented as a function of their distance.
%  [1] A. Goldsmith, Wireless Communications. Cambridge, U.K.: Cambridge Univ. Press, 2005.
%  ---------------------------------------------------------------------------------------------------------------------------------