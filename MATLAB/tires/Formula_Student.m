%% Tire Parameters: Formula Student        %%
% ----------------------------------------- %
% Version: V1.0 - 2024.09.14                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% ----------------------------------------- %

% General
tir.r_0 = 0.205;    % m
tir.b_0 = 0.1905;   % m
tir.c_x = 150E3;    % N/m
tir.c_y = 85E3;     % N/m
tir.c_z = 105E3;    % N/m
tir.c_psi = 400;    % Nm/rad
tir.d_x = 400;      % N*s/m
tir.d_y = 300;      % N*s/m
tir.d_z = 150;      % N*s/m
tir.d_psi = 1;      % Nm*s/rad
tir.a_r = 0.02;     % N/N
tir.dr_s= 1E-7;     % Tire radius increase over omega^2 in m/(rad/s)^2

% Tire model properties
tir.F_zN  = 575;    % Nominal normal force in N
tir.F_zMax= 4000;   % Maximum normal force in N
tir.R_N   = 0.05;   % Tire cross section roundness in [0,1]
tir.v_n   = 1;      % Velocity offset for slip calculation in m/s
fac_sS    = 1.67;   % Factor of s_M where full sliding starts in -
fac_FS    = 0.95;   % Ratio of sliding to peak force in -
fac_NoTz1 = 1.5;    % Factor of s_yM for first T_Align zero in -
fac_NoTz2 = 1.9;    % Factor of s_yM for second T_Align zero in -
effCamber = 1.0;    % Camber force effectiveness in -
effBore   = 0.7;    % Bore torque effectiveness in -

% F_x @ F_zN
tir.dF_x01  = 35E3;

tir.s_xMA1  = 0.13;
tir.F_xMA1  = 1500;
tir.s_xSA1  = tir.s_xMA1*fac_sS;
tir.F_xSA1  = tir.F_xMA1*fac_FS;

tir.s_xMB1  = 0.16;
tir.F_xMB1  = 1600;
tir.s_xSB1  = tir.s_xMB1*fac_sS;
tir.F_xSB1  = tir.F_xMB1*fac_FS;

% F_x @ 2*F_zN
tir.dF_x02  = 50E3;

tir.s_xMA2  = 0.13;
tir.F_xMA2  = 2600;
tir.s_xSA2  = tir.s_xMA2*fac_sS;
tir.F_xSA2  = tir.F_xMA2*fac_FS;

tir.s_xMB2  = 0.16;
tir.F_xMB2  = 2800;
tir.s_xSB2  = tir.s_xMB2*fac_sS;
tir.F_xSB2  = tir.F_xMB2*fac_FS;

% F_y @ F_zN
tir.dF_y01  = 25E3;
tir.s_yM1   = 0.18;
tir.F_yM1   = 1500;
tir.s_yS1   = tir.s_yM1*fac_sS;
tir.F_yS1   = tir.F_yM1*fac_FS;

% F_y @ 2*F_zN
tir.dF_y02  = 40E3;
tir.s_yM2   = 0.17;
tir.F_yM2   = 2600;
tir.s_yS2   = tir.s_yM2*fac_sS;
tir.F_yS2   = tir.F_yM2*fac_FS;

% Trail @ F_zN
tir.nu_01 = 0.40;
tir.s_y01 = tir.s_yM1*fac_NoTz1;
tir.s_yE1 = tir.s_yM1*fac_NoTz2;

% Trail @ 2*F_zN
tir.nu_02 = 0.45;
tir.s_y02 = tir.s_yM2*fac_NoTz1;
tir.s_yE2 = tir.s_yM2*fac_NoTz2;

% Other parameters
tir.eta_c = effCamber;
tir.eta_b = effBore;