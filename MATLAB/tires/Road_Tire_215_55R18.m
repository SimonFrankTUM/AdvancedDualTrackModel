%% Tire Parameters: Road Tire 215/55R18    %%
% ----------------------------------------- %
% Version: V1.0 - 2024.09.14                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% ----------------------------------------- %

% General
tir.r_0 = 0.3469;   % m
tir.b_0 = 0.215;    % m
tir.c_x = 250E3;    % N/m
tir.c_y = 175E3;    % N/m
tir.c_z = 200E3;    % N/m
tir.c_psi = 1000;   % Nm/rad
tir.d_x = 2000;     % N*s/m
tir.d_y = 1750;      % N*s/m
tir.d_z = 500;      % N*s/m
tir.d_psi = 3;      % Nm*s/rad
tir.a_r = 0.01;     % N/N
tir.dr_s= 1E-7;     % Tire radius increase over omega^2 in m/(rad/s)^2

% Tire model properties
tir.F_zN  = 3500;   % Nominal normal force in N
tir.F_zMax= 14000;  % Maximum normal force in N
tir.R_N   = 0.05;   % Tire cross section roundness in [0,1]
tir.v_n   = 0.5;    % Velocity offset for slip calculation in m/s
fac_sS    = 3.0;    % Factor of s_M where full sliding starts in -
fac_FS    = 0.90;   % Ratio of sliding to peak force in -
fac_NoTz1 = 1.5;    % Factor of s_yM for first T_Align zero in -
fac_NoTz2 = 2.0;    % Factor of s_yM for second T_Align zero in -
effCamber = 1.0;    % Camber force effectiveness in -
effBore   = 1.0;    % Bore torque effectiveness in -

% F_x @ F_zN
tir.dF_x01  = 100E3;

tir.s_xMA1  = 0.110;
tir.F_xMA1  = 3900;
tir.s_xSA1  = tir.s_xMA1*fac_sS;
tir.F_xSA1  = tir.F_xMA1*fac_FS;

tir.s_xMB1  = 0.121;
tir.F_xMB1  = 4290;
tir.s_xSB1  = tir.s_xMB1*fac_sS;
tir.F_xSB1  = tir.F_xMB1*fac_FS;

% F_x @ 2*F_zN
tir.dF_x02  = 190E3;

tir.s_xMA2  = 0.150;
tir.F_xMA2  = 7400;
tir.s_xSA2  = tir.s_xMA2*fac_sS;
tir.F_xSA2  = tir.F_xMA2*fac_FS;

tir.s_xMB2  = 0.165;
tir.F_xMB2  = 8140;
tir.s_xSB2  = tir.s_xMB2*fac_sS;
tir.F_xSB2  = tir.F_xMB2*fac_FS;

% F_y @ F_zN
tir.dF_y01  = 80E3;
tir.s_yM1   = 0.160;
tir.F_yM1   = 3650;
tir.s_yS1   = tir.s_yM1*fac_sS;
tir.F_yS1   = tir.F_yM1*fac_FS;

% F_y @ 2*F_zN
tir.dF_y02  = 152E3;
tir.s_yM2   = 0.210;
tir.F_yM2   = 6935;
tir.s_yS2   = tir.s_yM2*fac_sS;
tir.F_yS2   = tir.F_yM2*fac_FS;

% Trail @ F_zN
tir.nu_01 = 0.167;
tir.s_y01 = tir.s_yM1*fac_NoTz1;
tir.s_yE1 = tir.s_yM1*fac_NoTz2;

% Trail @ 2*F_zN
tir.nu_02 = 0.167;
tir.s_y02 = tir.s_yM2*fac_NoTz1;
tir.s_yE2 = tir.s_yM2*fac_NoTz2;

% Other parameters
tir.eta_c = effCamber;
tir.eta_b = effBore;