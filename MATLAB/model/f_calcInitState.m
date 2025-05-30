function [x_V0,y_V0,z_V0,phi_0,theta_0,phi_FL0,phi_FR0,phi_RL0,phi_RR0] = f_calcInitState(veh,tirFL,tirFR,tirRL,tirRR,env,road,opts)
%%      Calculation of initial states      %%
% ----------------------------------------- %
% Version: V1.4 - 2024.05.20                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
%   veh     -   struct containing vehicle   %
%               parameters                  %
%   tirXX   -   struct containing tire      %
%               parameters for one tire     %
%               where XX is FL/FR/RL/RR     %
%   env     -   struct containing           %
%               environment parameters      %
%   road    -   struct of road              %
%               x, y, and Z data            %
%   opts    -   struct of additional model  %
%               options                     %
% Output:                                   %
%   x_V0    -   value of x_V at t=0         %
%   y_V0    -   value of y_V at t=0         %
%   z_V0    -   value of z_V at t=0         %
%   phi_0   -   value of phi at t=0         %
%   theta_0 -   value of theta at t=0       %
%   phi_XX0 -   value of phi_XX at t=0      %
% ----------------------------------------- %

%% Initial states @ t=0
m_FA = veh.m*veh.l_RA/veh.wb;
m_RA = veh.m - m_FA;

% Road height under each wheel
if opts.use3DRoad % 3D road
    z_RFL = f_roadHeight(road, veh.l_FA, veh.t_FA*0.5,0);
    z_RFR = f_roadHeight(road, veh.l_FA,-veh.t_FA*0.5,0);
    z_RRL = f_roadHeight(road,-veh.l_RA, veh.t_RA*0.5,0);
    z_RRR = f_roadHeight(road,-veh.l_RA,-veh.t_RA*0.5,0);
else
    z_RFL = 0;
    z_RFR = 0;
    z_RRL = 0;
    z_RRR = 0;
end

% Lift on each axle
F_zAFA = 0.5*env.rho_Air*veh.C_LFA*veh.A_A*opts.x_0(15)^2; % Lift front
F_zARA = 0.5*env.rho_Air*veh.C_LRA*veh.A_A*opts.x_0(15)^2; % Lift rear

% Wheel travel from aerodynamic lift
z_AFA = F_zAFA/(2*veh.c_FA);
z_ARA = F_zARA/(2*veh.c_RA);

% Approx. initial wishbone angles from aerodynamic lift
phi_FL0 = asin(-z_AFA/( 0.5*veh.t_FA));
phi_FR0 = asin(-z_AFA/(-0.5*veh.t_FA));
phi_RL0 = asin(-z_ARA/( 0.5*veh.t_RA));
phi_RR0 = asin(-z_ARA/(-0.5*veh.t_RA));

% Force on each axle center
F_zFA = F_zAFA - env.g*m_FA;
F_zRA = F_zARA - env.g*m_RA;

% Wheel center heights
z_FL = 0.99*tirFL.r_0 + 0.5*F_zFA/tirFL.c_z + z_RFL;
z_FR = 0.99*tirFR.r_0 + 0.5*F_zFA/tirFR.c_z + z_RFR;
z_RL = 0.99*tirRL.r_0 + 0.5*F_zRA/tirRL.c_z + z_RRL;
z_RR = 0.99*tirRR.r_0 + 0.5*F_zRA/tirRR.c_z + z_RRR;

% Chassis angles
phi_0    = ( asin((z_FL-z_FR)/veh.t_FA) + asin((z_RL-z_RR)/veh.t_RA) )/2;
theta_0  = ( asin((z_RL-z_FL)/veh.wb)   + asin((z_RR-z_FR)/veh.wb) )/2 + ...
             asin((z_ARA-z_AFA)/veh.wb);

A_0V = f_yrot(theta_0) * f_xrot(phi_0);
z_VV = [0; 0; veh.z_cg + (z_AFA+z_ARA)/2 ...
    + 0.25*(z_FL+z_FR+z_RL+z_RR) - 0.25*(z_RFL+z_RFR+z_RRL+z_RRR)];
pos_0 = A_0V*z_VV + [0; 0; 0.25*(z_RFL+z_RFR+z_RRL+z_RRR)];

% CG position
x_V0 = pos_0(1);
y_V0 = pos_0(2);
z_V0 = pos_0(3);

end