function [x_dot, out] = f_vehicleModel(inputs, t, x, veh, tirFL, tirFR, tirRL, tirRR, env, road, opts)
%%      Vehicle Model Equations            %%
% ----------------------------------------- %
% Version: V1.16 - 2025.05.21               %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
%   inputs  -   struct of model input       %
%               values at time t            %
%   t       -   current simulation time     %
%   x       -   state vector                %
%   veh     -   struct containing vehicle   %
%               parameters                  %
%   tirXX   -   struct containing tire      %
%               parameters for one tire     %
%               where XX is FL/FR/RL/RR     %
%   env     -   struct containing           %
%               environment parameters      %
%   road    -   struct of road              %
%               x, y, Z, and Mu data        %
%   opts    -   struct of additional model  %
%               options                     %
% Output:                                   %
%   x_dot   -   state derivative vector     %
%   out     -   model output vector         %
% ----------------------------------------- %

%% Model states
% Generalized coordinates
y = x(1:14);
% Vehicle position
x_V=y(1); y_V=y(2); z_V=y(3);
% Euler angles
phi=y(4); theta=y(5); psi=y(6);
% Lower wishbone angles
phi_FL=y(7); phi_FR=y(8); phi_RL=y(9); phi_RR=y(10);
% Wheel rotation angles
rho_FL=y(11); rho_FR=y(12); rho_RL=y(13); rho_RR=y(14); %#ok<NASGU>

% Generalized velocities
z = x(15:28);
% Vehicle velocities
v_x=z(1); v_y=z(2); v_z=z(3);
% Vehicle angular velocities
omega_x=z(4); omega_y=z(5); omega_z=z(6);
% Lower wishbone angular velocities
phi_FL_dot=z(7); phi_FR_dot=z(8); phi_RL_dot=z(9); phi_RR_dot=z(10);
% Wheel rotation angular velocities 
omega_FL=z(11); omega_FR=z(12); omega_RL=z(13); omega_RR=z(14);

% Tire model internal states
x_TFL = x(29:31); x_TFR = x(32:34); x_TRL = x(35:37); x_TRR = x(38:40);

% Check states
if abs(phi) > 1.05
    error([num2str(t) 's: Roll angle too high (>60°).']);
elseif abs(theta) > 1.05
    error([num2str(t) 's: Pitch angle too high (>60°).']);
end

%% Model Input
delta_D = inputs.delta_D;
p_BFA   = inputs.p_BFA;
p_BRA   = inputs.p_BRA;
throttle= inputs.throttle;
gear    = inputs.gear;
mu_in   = inputs.mu_N;

%% Rotations
% Direction vectors
e_x = [1;0;0]; e_y = [0;1;0]; e_z = [0;0;1]; %#ok<NASGU>

% Chassis to earth
A_0V = f_zrot(psi) * f_yrot(theta) * f_xrot(phi);

% Velocities to coordinates
K_R  = [ 1  0          -sin(theta);
         0  cos(phi)    sin(phi)*cos(theta);
         0 -sin(phi)    cos(phi)*cos(theta) ];

K = eye(14);
K(1:3,1:3) = A_0V';
K(4:6,4:6) = K_R;

g_V = A_0V'*env.g*-e_z;

%% Suspension kinematics
% Front axle
u = f_limitAbs(veh.i_S*delta_D,veh.u_max); % Front steering rack travel
switch veh.susType_FA
    case 2 % Double-Wishbone
        [A_VFL,r_VFLV,delta_FL,dvdphi_FL,domdphi_FL,dvdu_FL,domdu_FL] = f_suspensionDWB(veh.susFL,phi_FL,u,t);
        [A_VFR,r_VFRV,delta_FR,dvdphi_FR,domdphi_FR,dvdu_FR,domdu_FR] = f_suspensionDWB(veh.susFR,phi_FR,u,t);

    case 1 % linear (phi is treated as z_W)
        i_u2del = veh.delta_max/veh.u_max; % Linear steering ratio (u -> delta)

        % dv Uprights
        dvdphi_FL   = e_z;
        dvdphi_FR   = e_z;
        dvdu_FL     = [0; 0; 0];
        dvdu_FR     = [0; 0; 0];

        % dom Uprights
        domdphi_FL  = [0; 0; 0];
        domdphi_FR  = [0; 0; 0];
        domdu_FL    = [0; 0; i_u2del];
        domdu_FR    = [0; 0; i_u2del];

        % Wheel center positions vehicle frame
        r_VFLV = [ veh.l_FA;  veh.t_FA/2; phi_FL-veh.z_cg];
        r_VFRV = [ veh.l_FA; -veh.t_FA/2; phi_FR-veh.z_cg];

        % Uprights to chassis
        delta_FL = i_u2del*u + veh.susFL.delta_0;
        delta_FR = i_u2del*u + veh.susFR.delta_0;
        A_VFL = f_zrot(delta_FL)*f_xrot(veh.susFL.gamma_0);
        A_VFR = f_zrot(delta_FR)*f_xrot(veh.susFR.gamma_0);
end

% Rear axle
switch veh.susType_RA
    case 2 % Double-Wishbone
        [A_VRL,r_VRLV,delta_RL,dvdphi_RL,domdphi_RL] = f_suspensionDWB(veh.susRL,phi_RL,0,t);
        [A_VRR,r_VRRV,delta_RR,dvdphi_RR,domdphi_RR] = f_suspensionDWB(veh.susRR,phi_RR,0,t);

    case 1 % linear (phi is treated as z_W)
        % dv Uprights
        dvdphi_RL   = e_z;
        dvdphi_RR   = e_z;

        % dom Uprights
        domdphi_RL  = [0; 0; 0];
        domdphi_RR  = [0; 0; 0];

        % Wheel center positions vehicle frame
        r_VRLV = [-veh.l_RA;  veh.t_RA/2; phi_RL-veh.z_cg];
        r_VRRV = [-veh.l_RA; -veh.t_RA/2; phi_RR-veh.z_cg];

        % Uprights to chassis
        delta_RL = 0;
        delta_RR = 0;
        A_VRL = f_zrot(veh.susRL.delta_0)*f_xrot(veh.susRL.gamma_0);
        A_VRR = f_zrot(veh.susRR.delta_0)*f_xrot(veh.susRR.gamma_0);
end

% Wheel rot. axis vehicle frame
e_yFLV = A_VFL*e_y;
e_yFRV = A_VFR*e_y;
e_yRLV = A_VRL*e_y;
e_yRRV = A_VRR*e_y;

% Wheel rot. axis origin frame
e_yFL0 = A_0V*e_yFLV;
e_yFR0 = A_0V*e_yFRV;
e_yRL0 = A_0V*e_yRLV;
e_yRR0 = A_0V*e_yRRV;

% Upright inertia tensor vehicle frame
I_UFLV = A_VFL*veh.I_UFL*A_VFL';
I_UFRV = A_VFR*veh.I_UFR*A_VFR';
I_URLV = A_VRL*veh.I_URL*A_VRL';
I_URRV = A_VRR*veh.I_URR*A_VRR';

% Wheel inertia tensor vehicle frame
I_WFLV = A_VFL*veh.I_WFL*A_VFL';
I_WFRV = A_VFR*veh.I_WFR*A_VFR';
I_WRLV = A_VRL*veh.I_WRL*A_VRL';
I_WRRV = A_VRR*veh.I_WRR*A_VRR';

%% Positions
r_0V0  = [x_V; y_V; z_V];
r_VFAV = [veh.l_FA;0;-veh.z_cg];
r_VRAV = [-veh.l_RA;0;-veh.z_cg];
r_0FL0 = r_0V0 + A_0V*r_VFLV;
r_0FR0 = r_0V0 + A_0V*r_VFRV;
r_0RL0 = r_0V0 + A_0V*r_VRLV;
r_0RR0 = r_0V0 + A_0V*r_VRRV;

%% Velocities
% Vehicle velocity vehicle frame
v_0VV     = [v_x; v_y; v_z];                % Vel. @ CoG
omega_0VV = [omega_x; omega_y; omega_z];    % Angular vel. @ CoG
v_0FAV    = v_0VV + cross(omega_0VV,r_VFAV); % Vel. @ front axle
v_FA      = sqrt(v_0FAV(1)^2 + v_0FAV(2)^2 + v_0FAV(3)^2);   % Norm of vel. @ front axle
v_0RAV    = v_0VV + cross(omega_0VV,r_VRAV); % Vel. @ rear axle
v_RA      = sqrt(v_0RAV(1)^2 + v_0RAV(2)^2 + v_0RAV(3)^2);   % Norm of vel. @ rear axle

% Wheel center velocity vehicle frame
r_VFLV_dot = dvdphi_FL*phi_FL_dot;
r_VFRV_dot = dvdphi_FR*phi_FR_dot;
r_VRLV_dot = dvdphi_RL*phi_RL_dot;
r_VRRV_dot = dvdphi_RR*phi_RR_dot;

% Wheel center total velocity vehicle frame
v_0FLV = v_0VV + cross(omega_0VV,r_VFLV) + r_VFLV_dot;
v_0FRV = v_0VV + cross(omega_0VV,r_VFRV) + r_VFRV_dot;
v_0RLV = v_0VV + cross(omega_0VV,r_VRLV) + r_VRLV_dot;
v_0RRV = v_0VV + cross(omega_0VV,r_VRRV) + r_VRRV_dot;

% Wheel center velocities origin frame
v_0FL0 = A_0V*v_0FLV;
v_0FR0 = A_0V*v_0FRV;
v_0RL0 = A_0V*v_0RLV;
v_0RR0 = A_0V*v_0RRV;

% Upright angular velocity vehicle frame
omega_VUFLV = domdphi_FL*phi_FL_dot;
omega_VUFRV = domdphi_FR*phi_FR_dot;
omega_VURLV = domdphi_RL*phi_RL_dot;
omega_VURRV = domdphi_RR*phi_RR_dot;

% Upright total angular velocity vehicle frame
omega_0UFLV = omega_0VV + omega_VUFLV;
omega_0UFRV = omega_0VV + omega_VUFRV;
omega_0URLV = omega_0VV + omega_VURLV;
omega_0URRV = omega_0VV + omega_VURRV;

% Upright angular velocity origin frame
omega_0UFL0 = A_0V*omega_0UFLV;
omega_0UFR0 = A_0V*omega_0UFRV;
omega_0URL0 = A_0V*omega_0URLV;
omega_0URR0 = A_0V*omega_0URRV;

% Wheel angular velocity vehicle frame
omega_0WFLV = omega_0UFLV + e_yFLV*omega_FL;
omega_0WFRV = omega_0UFRV + e_yFRV*omega_FR;
omega_0WRLV = omega_0URLV + e_yRLV*omega_RL;
omega_0WRRV = omega_0URRV + e_yRRV*omega_RR;

% Wheel angular velocity wheel frame
omega_0WFLW = A_VFL'*omega_0WFLV;
omega_0WFRW = A_VFR'*omega_0WFRV;
omega_0WRLW = A_VRL'*omega_0WRLV;
omega_0WRRW = A_VRR'*omega_0WRRV;

%% Partial velocities
% Partial velocities
dvVdz  = [eye(3) zeros(3,11)];
dvFLdz = [eye(3) f_crossprod(r_VFLV)' dvdphi_FL zeros(3,7)];
dvFRdz = [eye(3) f_crossprod(r_VFRV)' zeros(3,1) dvdphi_FR zeros(3,6)];
dvRLdz = [eye(3) f_crossprod(r_VRLV)' zeros(3,2) dvdphi_RL zeros(3,5)];
dvRRdz = [eye(3) f_crossprod(r_VRRV)' zeros(3,3) dvdphi_RR zeros(3,4)];

% Partial angular velocities
domVdz = [zeros(3) eye(3) zeros(3,8)];

% Wheels
domWFLdz = [zeros(3) eye(3)-e_yFLV*e_yFLV' domdphi_FL zeros(3,3) e_yFLV zeros(3,3)];
domWFRdz = [zeros(3) eye(3)-e_yFRV*e_yFRV' zeros(3,1) domdphi_FR zeros(3,3) e_yFRV zeros(3,2)];
domWRLdz = [zeros(3) eye(3)-e_yRLV*e_yRLV' zeros(3,2) domdphi_RL zeros(3,3) e_yRLV zeros(3,1)];
domWRRdz = [zeros(3) eye(3)-e_yRRV*e_yRRV' zeros(3,3) domdphi_RR zeros(3,3) e_yRRV];

% Uprights
domUFLdz = [zeros(3) eye(3) domdphi_FL zeros(3,7)];
domUFRdz = [zeros(3) eye(3) zeros(3,1) domdphi_FR zeros(3,6)];
domURLdz = [zeros(3) eye(3) zeros(3,2) domdphi_RL zeros(3,5)];
domURRdz = [zeros(3) eye(3) zeros(3,3) domdphi_RR zeros(3,4)];

%% Tire road contact
[v_0QFLQ,A_0QFL,gamma_FL,r_stFL,dz_FL,w_FL,r_MQFL0,mu_RFL] = f_roadContact(road,tirFL,r_0FL0,v_0FL0,omega_0UFL0,e_yFL0,opts,t);
[v_0QFRQ,A_0QFR,gamma_FR,r_stFR,dz_FR,w_FR,r_MQFR0,mu_RFR] = f_roadContact(road,tirFR,r_0FR0,v_0FR0,omega_0UFR0,e_yFR0,opts,t);
[v_0QRLQ,A_0QRL,gamma_RL,r_stRL,dz_RL,w_RL,r_MQRL0,mu_RRL] = f_roadContact(road,tirRL,r_0RL0,v_0RL0,omega_0URL0,e_yRL0,opts,t);
[v_0QRRQ,A_0QRR,gamma_RR,r_stRR,dz_RR,w_RR,r_MQRR0,mu_RRR] = f_roadContact(road,tirRR,r_0RR0,v_0RR0,omega_0URR0,e_yRR0,opts,t);

A_VQFL = A_0V'*A_0QFL;
A_VQFR = A_0V'*A_0QFR;
A_VQRL = A_0V'*A_0QRL;
A_VQRR = A_0V'*A_0QRR;

% Tire forces
switch opts.tireModel
    case 2 % TMeasy
        [F_TFL,T_TFL,s_FL,x_TFL_dot,r_dynFL,F_ycFL,dF_x0FL,F_xMFL,dF_y0FL,F_yMFL] = ...
            f_tireTMeasy(tirFL,t,v_0QFLQ,omega_0WFLW,r_stFL,dz_FL,w_FL,gamma_FL,env.mu_N*mu_in*mu_RFL,x_TFL,opts.tireDyn);
        [F_TFR,T_TFR,s_FR,x_TFR_dot,r_dynFR,F_ycFR,dF_x0FR,F_xMFR,dF_y0FR,F_yMFR] = ...
            f_tireTMeasy(tirFR,t,v_0QFRQ,omega_0WFRW,r_stFR,dz_FR,w_FR,gamma_FR,env.mu_N*mu_in*mu_RFR,x_TFR,opts.tireDyn);
        [F_TRL,T_TRL,s_RL,x_TRL_dot,r_dynRL,F_ycRL,dF_x0RL,F_xMRL,dF_y0RL,F_yMRL] = ...
            f_tireTMeasy(tirRL,t,v_0QRLQ,omega_0WRLW,r_stRL,dz_RL,w_RL,gamma_RL,env.mu_N*mu_in*mu_RRL,x_TRL,opts.tireDyn);
        [F_TRR,T_TRR,s_RR,x_TRR_dot,r_dynRR,F_ycRR,dF_x0RR,F_xMRR,dF_y0RR,F_yMRR] = ...
            f_tireTMeasy(tirRR,t,v_0QRRQ,omega_0WRRW,r_stRR,dz_RR,w_RR,gamma_RR,env.mu_N*mu_in*mu_RRR,x_TRR,opts.tireDyn);
        x_T_dot = [x_TFL_dot; x_TFR_dot; x_TRL_dot; x_TRR_dot];
    case 1 % linear
        [F_TFL,T_TFL,s_FL,r_dynFL,F_ycFL,dF_x0FL,F_xMFL,dF_y0FL,F_yMFL] = ...
            f_tireLinear(tirFL,t,v_0QFLQ,omega_0WFLW,r_stFL,dz_FL,env.mu_N*mu_in*mu_RFL);
        [F_TFR,T_TFR,s_FR,r_dynFR,F_ycFR,dF_x0FR,F_xMFR,dF_y0FR,F_yMFR] = ...
            f_tireLinear(tirFR,t,v_0QFRQ,omega_0WFRW,r_stFR,dz_FR,env.mu_N*mu_in*mu_RFR);
        [F_TRL,T_TRL,s_RL,r_dynRL,F_ycRL,dF_x0RL,F_xMRL,dF_y0RL,F_yMRL] = ...
            f_tireLinear(tirRL,t,v_0QRLQ,omega_0WRLW,r_stRL,dz_RL,env.mu_N*mu_in*mu_RRL);
        [F_TRR,T_TRR,s_RR,r_dynRR,F_ycRR,dF_x0RR,F_xMRR,dF_y0RR,F_yMRR] = ...
            f_tireLinear(tirRR,t,v_0QRRQ,omega_0WRRW,r_stRR,dz_RR,env.mu_N*mu_in*mu_RRR);
        x_TFL_dot = [0;0;0]; x_TFR_dot = [0;0;0]; x_TRL_dot = [0;0;0]; x_TRR_dot = [0;0;0];
        x_T_dot = [x_TFL_dot; x_TFR_dot; x_TRL_dot; x_TRR_dot];
end

%% Suspension forces
z_FL = r_VFLV(3)+veh.z_cg;  z_FR = r_VFRV(3)+veh.z_cg;
z_RL = r_VRLV(3)+veh.z_cg;  z_RR = r_VRRV(3)+veh.z_cg;
z_FL_dot = r_VFLV_dot(3);   z_FR_dot = r_VFRV_dot(3);
z_RL_dot = r_VRLV_dot(3);   z_RR_dot = r_VRRV_dot(3);

% Suspension corner spring & damper forces
F_CrSFL = (z_FL)*veh.c_FA + f_LUT1D(veh.v_DFA,veh.F_DFA,z_FL_dot,true);
F_CrSFR = (z_FR)*veh.c_FA + f_LUT1D(veh.v_DFA,veh.F_DFA,z_FR_dot,true);
F_CrSRL = (z_RL)*veh.c_RA + f_LUT1D(veh.v_DRA,veh.F_DRA,z_RL_dot,true);
F_CrSRR = (z_RR)*veh.c_RA + f_LUT1D(veh.v_DRA,veh.F_DRA,z_RR_dot,true);

% Suspension heave spring & damper forces
F_HvSFA = 0.5*(z_FL+z_FR)*veh.c_HeaveFA + f_LUT1D(veh.v_DHeaveFA,veh.F_DHeaveFA,0.5*(z_FL_dot+z_FR_dot),true);
F_HvSRA = 0.5*(z_RL+z_RR)*veh.c_HeaveRA + f_LUT1D(veh.v_DHeaveRA,veh.F_DHeaveRA,0.5*(z_RL_dot+z_RR_dot),true);

% Suspension roll spring & damper forces
F_RoSFA = (z_FL-z_FR)*veh.c_RollFA + f_LUT1D(veh.v_DRollFA,veh.F_DRollFA,z_FL_dot-z_FR_dot,true);
F_RoSRA = (z_RL-z_RR)*veh.c_RollRA + f_LUT1D(veh.v_DRollRA,veh.F_DRollRA,z_RL_dot-z_RR_dot,true);

% Pretension + spring & damper forces
F_SFL = veh.F_SFL0 + F_CrSFL + F_RoSFA + 0.5*F_HvSFA;
F_SFR = veh.F_SFR0 + F_CrSFR - F_RoSFA + 0.5*F_HvSFA;
F_SRL = veh.F_SRL0 + F_CrSRL + F_RoSRA + 0.5*F_HvSRA;
F_SRR = veh.F_SRR0 + F_CrSRR - F_RoSRA + 0.5*F_HvSRA;

%% Aerodynamic Forces
F_DAFA = 0.5*env.rho_Air*veh.C_D*0.5*veh.A_A*v_FA*v_0FAV;  % Drag front
F_DARA = 0.5*env.rho_Air*veh.C_D*0.5*veh.A_A*v_RA*v_0RAV;  % Drag rear
F_LAFA = 0.5*env.rho_Air*veh.C_LFA*veh.A_A*e_z*v_0FAV(1)^2; % Lift front
F_LARA = 0.5*env.rho_Air*veh.C_LRA*veh.A_A*e_z*v_0RAV(1)^2; % Lift rear
F_AFAV = F_DAFA + F_LAFA; % Aero. force front
F_ARAV = F_DARA + F_LARA; % Aero. force rear

%% Applied forces & torques
% Wheel rot. friction damping coef.
d_NFA = tirFL.r_0*sqrt(tirFL.c_x*veh.I_WFL(2,2));
d_NRA = tirRL.r_0*sqrt(tirRL.c_x*veh.I_WRL(2,2));

% Drive torques
switch veh.Drive
    case 1 % RWD-LSD
        T_DFL = 0; T_DFR = 0;

        % Motor
        switch veh.Motor
            case 1 % Combustion
                i_G = f_LUT1D(veh.n_G,veh.i_G,gear,false);
                omega_M = 0.5*(omega_RL+omega_RR)*veh.i_F*i_G;
                if omega_M<veh.omega_Mmin
                    omega_M = veh.omega_Mmin;
                end

                facOmegaMLim = f_limitAbs(0.1*(veh.omega_Mmax-omega_M),1);
                T_MMax  = (env.rho_Air/1.2041) * f_LUT1D(veh.omega_M,veh.T_M,omega_M,false);
                facDrive= (omega_M-veh.omega_Mmin)/(abs(omega_M)+tirRL.v_n/tirRL.r_0); % Is 0 at standstill
                facTrq  = (1-facDrive)*throttle^veh.C_expTh + facDrive*(-veh.C_MBrake + (1+veh.C_MBrake)*throttle^veh.C_expTh);
                T_M     = facTrq*facOmegaMLim*T_MMax;

            case 2 % Electric
                i_G = veh.i_G(1);
                omega_M = 0.5*(omega_RL+omega_RR)*veh.i_F*i_G;
                facOmegaMLim = f_limitAbs(0.1*(veh.omega_Mmax-omega_M),1);
                T_MMax  = f_limitAbs(veh.T_MMax,veh.P_MMax/abs(omega_M));
                T_M     = throttle*facOmegaMLim*T_MMax;
        end

        % Drive torque at Gearbox output
        T_D = veh.i_F*i_G*(veh.eta_G*T_M - omega_M*veh.d_G);
        
        % LSD
        domega_RA = omega_RR - omega_RL;
        T_LMax = veh.T_LPL + abs(T_D)*veh.C_LSD(2+sign(T_D));
        T_L = f_limitAbs(domega_RA*d_NRA, T_LMax);
        T_DRL = 0.5*(T_D + T_L);
        T_DRR = 0.5*(T_D - T_L);

        % Brake torques
        T_BFL = f_limitAbs(-d_NFA*omega_FL, p_BFA*veh.i_p2TBFA);
        T_BFR = f_limitAbs(-d_NFA*omega_FR, p_BFA*veh.i_p2TBFA);
        T_BRL = f_limitAbs(-d_NRA*omega_RL, p_BRA*veh.i_p2TBRA);
        T_BRR = f_limitAbs(-d_NRA*omega_RR, p_BRA*veh.i_p2TBRA);

        % Applied torques
        % Wheels
        T_WFL = e_yFLV*T_BFL + e_y*T_DFL ...
            + cross(A_0V'*r_MQFL0,A_VQFL*F_TFL) ...
            + A_VQFL*T_TFL; % Wheel FL
        T_WFR = e_yFRV*T_BFR + e_y*T_DFR ...
            + cross(A_0V'*r_MQFR0,A_VQFR*F_TFR) ...
            + A_VQFR*T_TFR; % Wheel FR
        T_WRL = e_yRLV*T_BRL + e_y*T_DRL ...
            + cross(A_0V'*r_MQRL0,A_VQRL*F_TRL) ...
            + A_VQRL*T_TRL; % Wheel RL
        T_WRR = e_yRRV*T_BRR + e_y*T_DRR ...
            + cross(A_0V'*r_MQRR0,A_VQRR*F_TRR) ...
            + A_VQRR*T_TRR; % Wheel RR

        % Uprights
        T_UFL = e_yFLV*-T_BFL; % Upright FL
        T_UFR = e_yFRV*-T_BFR; % Upright FR
        T_URL = e_yRLV*-T_BRL; % Upright RL
        T_URR = e_yRRV*-T_BRR; % Upright RR

        % Chassis
        T_C = cross(r_VFLV,F_SFL*e_z) ...
            + cross(r_VFRV,F_SFR*e_z) ...
            + cross(r_VRLV,F_SRL*e_z) ...
            + cross(r_VRRV,F_SRR*e_z) ...
            + cross(r_VFAV,F_AFAV) ...
            + cross(r_VRAV,F_ARAV)...
            - e_y*( T_DFL + T_DFR ...
            + T_DRL + T_DRR );

    case 2 % Hub-Motors
        if veh.Motor ~= 2
            error('Only electric motors are supported as hub motors.')
        end

        facOmegaMLimFL = f_limitAbs(0.1*(veh.omega_Mmax/veh.i_G(1)-omega_FL),1);
        facOmegaMLimFR = f_limitAbs(0.1*(veh.omega_Mmax/veh.i_G(1)-omega_FR),1);
        facOmegaMLimRL = f_limitAbs(0.1*(veh.omega_Mmax/veh.i_G(1)-omega_RL),1);
        facOmegaMLimRR = f_limitAbs(0.1*(veh.omega_Mmax/veh.i_G(1)-omega_RR),1);

        % All wheel drive torque control
        [T_MCmdFL,T_MCmdFR,T_MCmdRL,T_MCmdRR] = f_eMotTorqueCtrl(veh,tirFL,tirFR,tirRL,tirRR,x,throttle);

        T_MFL = facOmegaMLimFL*sign(T_MCmdFL)*min([veh.T_MMax veh.P_MMax*veh.eta_M/abs(omega_FL) abs(T_MCmdFL)]);
        T_MFR = facOmegaMLimFR*sign(T_MCmdFR)*min([veh.T_MMax veh.P_MMax*veh.eta_M/abs(omega_FR) abs(T_MCmdFR)]);
        T_MRL = facOmegaMLimRL*sign(T_MCmdRL)*min([veh.T_MMax veh.P_MMax*veh.eta_M/abs(omega_RL) abs(T_MCmdRL)]);
        T_MRR = facOmegaMLimRR*sign(T_MCmdRR)*min([veh.T_MMax veh.P_MMax*veh.eta_M/abs(omega_RR) abs(T_MCmdRR)]);

        T_DFL = veh.i_G(1)*veh.eta_G*T_MFL - omega_FL*veh.d_G;
        T_DFR = veh.i_G(1)*veh.eta_G*T_MFR - omega_FR*veh.d_G;
        T_DRL = veh.i_G(1)*veh.eta_G*T_MRL - omega_RL*veh.d_G;
        T_DRR = veh.i_G(1)*veh.eta_G*T_MRR - omega_RR*veh.d_G;

        % No single motor
        omega_M = 0;
        T_M     = 0;

        % Brake torques
        T_BFL = f_limitAbs(-d_NFA*omega_FL, p_BFA*veh.i_p2TBFA);
        T_BFR = f_limitAbs(-d_NFA*omega_FR, p_BFA*veh.i_p2TBFA);
        T_BRL = f_limitAbs(-d_NRA*omega_RL, p_BRA*veh.i_p2TBRA);
        T_BRR = f_limitAbs(-d_NRA*omega_RR, p_BRA*veh.i_p2TBRA);

        % Applied torques
        % Wheels
        T_WFL = e_yFLV*(T_BFL + T_DFL) ...
            + cross(A_0V'*r_MQFL0,A_VQFL*F_TFL) ...
            + A_VQFL*T_TFL; % Wheel FL
        T_WFR = e_yFRV*(T_BFR + T_DFR) ...
            + cross(A_0V'*r_MQFR0,A_VQFR*F_TFR) ...
            + A_VQFR*T_TFR; % Wheel FR
        T_WRL = e_yRLV*(T_BRL + T_DRL) ...
            + cross(A_0V'*r_MQRL0,A_VQRL*F_TRL) ...
            + A_VQRL*T_TRL; % Wheel RL
        T_WRR = e_yRRV*(T_BRR + T_DRR) ...
            + cross(A_0V'*r_MQRR0,A_VQRR*F_TRR) ...
            + A_VQRR*T_TRR; % Wheel RR

        % Uprights
        T_UFL = -e_yFLV*(T_BFL + T_DFL); % Upright FL
        T_UFR = -e_yFRV*(T_BFR + T_DFR); % Upright FR
        T_URL = -e_yRLV*(T_BRL + T_DRL); % Upright RL
        T_URR = -e_yRRV*(T_BRR + T_DRR); % Upright RR

        % Chassis
        T_C = cross(r_VFLV,F_SFL*e_z) ...
            + cross(r_VFRV,F_SFR*e_z) ...
            + cross(r_VRLV,F_SRL*e_z) ...
            + cross(r_VRRV,F_SRR*e_z) ...
            + cross(r_VFAV,F_AFAV) ...
            + cross(r_VRAV,F_ARAV);
end

% Applied forces
F_C  = F_AFAV+F_ARAV + (F_SFL+F_SFR+F_SRL+F_SRR)*e_z; % Chassis
F_FL = A_VQFL*F_TFL - F_SFL*e_z; % FL
F_FR = A_VQFR*F_TFR - F_SFR*e_z; % FR
F_RL = A_VQRL*F_TRL - F_SRL*e_z; % RL
F_RR = A_VQRR*F_TRR - F_SRR*e_z; % RR

% Applied force vector
f =   dvVdz' * ( F_C ) ...      % Chassis force
    + domVdz'* ( T_C ) ...      % Chassis torque
    + dvFLdz' * ( F_FL ) ...    % Force FL
    + dvFRdz' * ( F_FR ) ...    % Force FR
    + dvRLdz' * ( F_RL ) ...    % Force RL
    + dvRRdz' * ( F_RR ) ...    % Force RR
    + domUFLdz' * ( T_UFL ) ... % Upright torque FL
    + domUFRdz' * ( T_UFR ) ... % Upright torque FR
    + domURLdz' * ( T_URL ) ... % Upright torque RL
    + domURRdz' * ( T_URR ) ... % Upright torque RR
    + domWFLdz' * ( T_WFL ) ... % Wheel torque FL
    + domWFRdz' * ( T_WFR ) ... % Wheel torque FR
    + domWRLdz' * ( T_WRL ) ... % Wheel torque RL
    + domWRRdz' * ( T_WRR );    % Wheel torque RR

% Generalized force vector
q = f ...
    + dvVdz'  * veh.m_C*( g_V - cross(omega_0VV,v_0VV) ) ... % Chassis force
    + domVdz' * ( - cross(omega_0VV,veh.I_C*omega_0VV) ) ... % Chassis torque
    + dvFLdz' * veh.m_FL*( g_V - ( cross(omega_0VV,r_VFLV_dot) + cross(omega_0VV,v_0FLV)) ) ... % Force FL
    + dvFRdz' * veh.m_FR*( g_V - ( cross(omega_0VV,r_VFRV_dot) + cross(omega_0VV,v_0FRV)) ) ... % Force FR
    + dvRLdz' * veh.m_RL*( g_V - ( cross(omega_0VV,r_VRLV_dot) + cross(omega_0VV,v_0RLV)) ) ... % Force RL
    + dvRRdz' * veh.m_RR*( g_V - ( cross(omega_0VV,r_VRRV_dot) + cross(omega_0VV,v_0RRV)) ) ... % Force RR
    + domUFLdz' * ( - cross(omega_0UFLV,I_UFLV*omega_0UFLV) ) ... % Upright torque FL
    + domUFRdz' * ( - cross(omega_0UFRV,I_UFRV*omega_0UFRV) ) ... % Upright torque FR
    + domURLdz' * ( - cross(omega_0URLV,I_URLV*omega_0URLV) ) ... % Upright torque RL
    + domURRdz' * ( - cross(omega_0URRV,I_URRV*omega_0URRV) ) ... % Upright torque RR
    + domWFLdz' * ( - I_WFLV*( cross(omega_0VV,omega_0WFLV) + cross(omega_VUFLV,e_yFLV*omega_FL) ) - cross(omega_0WFLV,I_WFLV*omega_0WFLV) ) ... % Wheel torque FL
    + domWFRdz' * ( - I_WFRV*( cross(omega_0VV,omega_0WFRV) + cross(omega_VUFRV,e_yFRV*omega_FR) ) - cross(omega_0WFRV,I_WFRV*omega_0WFRV) ) ... % Wheel torque FR
    + domWRLdz' * ( - I_WRLV*( cross(omega_0VV,omega_0WRLV) + cross(omega_VURLV,e_yRLV*omega_RL) ) - cross(omega_0WRLV,I_WRLV*omega_0WRLV) ) ... % Wheel torque RL
    + domWRRdz' * ( - I_WRRV*( cross(omega_0VV,omega_0WRRV) + cross(omega_VURRV,e_yRRV*omega_RR) ) - cross(omega_0WRRV,I_WRRV*omega_0WRRV) );    % Wheel torque RR

%% Generalized mass matrix
M =   (dvVdz'*veh.m_C*dvVdz + domVdz'*veh.I_C*domVdz) ... % Chassis
    + (dvFLdz'*veh.m_FL*dvFLdz + domUFLdz'*I_UFLV*domUFLdz + domWFLdz'*I_WFLV*domWFLdz) ... % FL
    + (dvFRdz'*veh.m_FR*dvFRdz + domUFRdz'*I_UFRV*domUFRdz + domWFRdz'*I_WFRV*domWFRdz) ... % FR
    + (dvRLdz'*veh.m_RL*dvRLdz + domURLdz'*I_URLV*domURLdz + domWRLdz'*I_WRLV*domWRLdz) ... % RL
    + (dvRRdz'*veh.m_RR*dvRRdz + domURRdz'*I_URRV*domURRdz + domWRRdz'*I_WRRV*domWRRdz);    % RR

%% Solve for state derivatives
y_dot = K \ z; % Generalized coordinates derivatives
z_dot = M \ q; % Generalized velocities derivatives
x_dot = [y_dot; z_dot; x_T_dot]; % State derivatives

%% Additional output
% Inertial acceleration
a_Inertial = M(1:3,1:3) \ f(1:3);

% Combined aerodynamic forces
F_A = F_AFAV + F_ARAV;

% Steering Torque
T_S = ( dvdu_FL' * ( F_FL ) ...
      + dvdu_FR' * ( F_FR ) ...
      + domdu_FL' * ( T_UFL + T_WFL ) ...
      + domdu_FR' * ( T_UFR + T_WFR ) ...
      + dvdu_FL' * veh.m_FL*( g_V - ( cross(omega_0VV,r_VFLV_dot) + cross(omega_0VV,v_0FLV)) ) ...
      + dvdu_FR' * veh.m_FR*( g_V - ( cross(omega_0VV,r_VFRV_dot) + cross(omega_0VV,v_0FRV)) ) ...
      + domdu_FL' * ( - cross(omega_0UFLV,I_UFLV*omega_0UFLV) ) ...
      + domdu_FR' * ( - cross(omega_0UFRV,I_UFRV*omega_0UFRV) ) ...
      + domdu_FL' * ( - I_WFLV*( cross(omega_0VV,omega_0WFLV) + cross(omega_VUFLV,e_yFLV*omega_FL) ) - cross(omega_0WFLV,I_WFLV*omega_0WFLV) ) ...
      + domdu_FR' * ( - I_WFRV*( cross(omega_0VV,omega_0WFRV) + cross(omega_VUFRV,e_yFRV*omega_FR) ) - cross(omega_0WFRV,I_WFRV*omega_0WFRV) ) ...
      )*veh.i_S;

% Output vector
out = [ a_Inertial; ...
    F_TFL;      F_TFR;      F_TRL;      F_TRR;      ...
    s_FL;       s_FR;       s_RL;       s_RR;       ...
    F_ycFL;     F_ycFR;     F_ycRL;     F_ycRR;     ...
    dF_x0FL;    dF_x0FR;    dF_x0RL;    dF_x0RR;    ...
    F_xMFL;     F_xMFR;     F_xMRL;     F_xMRR;     ...
    dF_y0FL;    dF_y0FR;    dF_y0RL;    dF_y0RR;    ...
    F_yMFL;     F_yMFR;     F_yMRL;     F_yMRR;     ...
    r_stFL;     r_stFR;     r_stRL;     r_stRR;     ...
    r_dynFL;    r_dynFR;    r_dynRL;    r_dynRR;    ...
    z_FL;       z_FR;       z_RL;       z_RR;       ...
    z_FL_dot;   z_FR_dot;   z_RL_dot;   z_RR_dot;   ...
    F_SFL;      F_SFR;      F_SRL;      F_SRR;      ...
    delta_FL;   delta_FR;   delta_RL;   delta_RR;   ...
    gamma_FL;   gamma_FR;   gamma_RL;   gamma_RR;   ...
    T_BFL;      T_BFR;      T_BRL;      T_BRR;      ...
    T_DFL;      T_DFR;      T_DRL;      T_DRR;      ...
    omega_M;    T_M;        F_A;        T_S];

end