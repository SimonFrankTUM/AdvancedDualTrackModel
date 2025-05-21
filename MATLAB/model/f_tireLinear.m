function [F_T,T_T,s,r_dyn,F_yc,dF_x0,F_xM,dF_y0,F_yM] = f_tireLinear(tir,t,v_0QQ,omega_0WW,r_st,dz,mu_n)
%%          Linear Tire Model              %%
% ----------------------------------------- %
% Version: V1.4 - 2024.07.09                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
%   tir     -   struct containing tire      %
%               parameters for one tire     %
%               where XX is FL/FR/RL/RR     %
%   t       -   current simulation time     %
%   v_0QQ   -   velocity vector of contact  %
%               point Q in relation to      %
%               origin in Q cosy.           %
%   omega_0WW - angular velocity vector     %
%               of wheel in relation to     %
%               origin in wheel frame       %
%   r_st    -   static tire radius          %
%   dz      -   vertical tire deflection    %
%   mu_n    -   Normalized coefficient of   %
%               friction                    %
% Output:                                   %
%   F_T     -   tire force vector           %
%   T_T     -   tire torque vector          %
%   s       -   tire slip vector            %
%   r_dyn   -   dynamic tire rolling radius %
%   F_yc    -   side force due to camber    %
%   dF_x0   -   Longitudinal slip stiffness %
%   F_xM    -   Maximum possible F_x        %
%   dF_y0   -   Lateral slip stiffness      %
%   F_yM    -   Maximum possible F_y        %
% ----------------------------------------- %

%% Deflection
% Dynamic rolling radius
r_dyn = tir.r_0*2/3 + r_st/3;

% Transport velocity
v_t = omega_0WW(2)*r_dyn;

%% Slip
s_x = (v_t - v_0QQ(1)) /(abs(v_t) + tir.v_n); % Longitudinal slip
s_y =      - v_0QQ(2)  /(abs(v_t) + tir.v_n); % Lateral slip
s   = [s_x; s_y; 0; 0];

% Generalized slip
s_N  = sqrt(s_x^2 + s_y^2);

if s_N > 0
    cosphi = s_x/s_N;
    sinphi = s_y/s_N;
else
    cosphi = 0;
    sinphi = 1;
end

%% Forces
F_z = dz*tir.c_z - v_0QQ(3)*tir.d_z;

if F_z <= 0
    F_T = zeros(3,1);
    T_T = zeros(3,1);
    s   = zeros(4,1);
    F_yc  = 0;
    dF_x0 = 0;
    F_xM  = 0;
    dF_y0 = 0;
    F_yM  = 0;
    return
elseif F_z > tir.F_zMax
    warning([num2str(t) 's: Tire normal load very high (' num2str(F_z) 'N).'],'backtrace','off');
end

% Tire force
mu = mu_n*(tir.F_xMA1+tir.F_yM1)/(2*tir.F_zN);
F_M = F_z*mu;
f   = 0.5*(tir.dF_x01+tir.dF_y01);

F_0 = f*s_N;
F   = f_limitAbs(F_0,F_M);
F_yc= 0;

% Split combined force into x and y components
F_x = F*cosphi;
F_y = F*sinphi;

% Tire force vector
F_T = [F_x; F_y; F_z];

% Additional outputs
dF_x0 = f;
dF_y0 = f;
F_xM  = F_M;
F_yM  = F_M;

%% Torques
T_x = 0;
T_y = F_z*tir.a_r*tir.r_0 * -v_t/(abs(v_t)+tir.v_n);
T_z = 0;

% Tire torque vector
T_T = [T_x; T_y; T_z];

end