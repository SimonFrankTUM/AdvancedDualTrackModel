function [s_x,s_y,s_b,r_dyn,v_T,F_x,F_y,F_z,T_x,T_y,T_z,x_e_dot,y_e_dot,psi_e_dot] = ...
    TMeasyExternal(tir,r_st,v_x,v_y,v_z,omega_y,omega_z,gamma,mu_n,x_e,y_e,psi_e,dynamic)
%%          TMeasy Tire Model              %%
% ----------------------------------------- %
% Version: V1.0 - 2025.06.09                %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info:                                     %
%   The tire deflection states x_e, y_e,    %
%   and psi_e must be integrated in the     %
%   simulation model if dynamic is set true.%
% Input:                                    %
%   tir     -   struct containing tire      %
%               parameters for one tire     %
%   r_st    -   static tire radius          %
%   v_x/y/z -   tire contact velocity       %
%   omega_y/z - angular wheel velocity      %
%   gamma   -   inclination angle ("camber")%
%   mu_n    -   normalized coefficient of   %
%               friction                    %
%   x/y/psi_e - tire deflection states      %
%   dynamic -   Enable/disable first order  %
%               dynamics (true/false)       %
% Output:                                   %
%   s_x/y/b -   tire slips                  %
%   r_dyn   -   dynamic tire rolling radius %
%   v_T     -   tread transport velocity    %
%   F_x/y/z -   tire forces                 %
%   T_x/y/z -   tire torques                %
%   x/y/psi_e_dot - tire state derivatives  %
% ----------------------------------------- %

%% Preparation
r_0     = tir.r_0 + tir.dr_s*omega_y^2; % Unloaded radius
r_dyn   = r_0*2/3 + r_st/3;     % Dynamic rolling radius
v_T     = omega_y*r_dyn;        % Transport velocity
v_TStar = abs(v_T) + tir.v_n;   % Absolute transport velocity + v_n

%% Tire deflection
% Rectangular cross section
r_stL = r_st + tir.b_0/2 * tan(gamma);
r_stR = r_st - tir.b_0/2 * tan(gamma);

if r_stL < tir.r_0 && r_stR < tir.r_0
    % Full contact
    dz_rect  =  tir.r_0 - r_st;
    w_rect   =  tir.b_0/cos(gamma);
    y_Q_rect = -tir.b_0^2*tan(gamma) / (12*dz_rect*cos(gamma));

elseif r_stL >= tir.r_0 && r_stR < tir.r_0 && gamma ~= 0
    % Left edge lifts off road
    dz_rect  = (tir.r_0-r_stR)^2 / (2*tir.b_0*tan(abs(gamma)));
    w_rect   = (tir.r_0-r_stR) / tan(abs(gamma));
    y_Q_rect = -sign(gamma)*(tir.b_0/(2*cos(gamma)) - w_rect/3);

elseif r_stL < tir.r_0 && r_stR >= tir.r_0 && gamma ~= 0
    % Right edge lifts off road
    dz_rect  = (tir.r_0-r_stL)^2 / (2*tir.b_0*tan(abs(gamma)));
    w_rect   = (tir.r_0-r_stL) / tan(abs(gamma));
    y_Q_rect = -sign(gamma)*(tir.b_0/(2*cos(gamma)) - w_rect/3);

else
    % No contact
    s_x = 0; s_y = 0; s_b = 0;
    F_x = 0; F_y = 0; F_z = 0;
    T_x = 0; T_y = 0; T_z = 0;
    x_e_dot     = (-tir.c_x*x_e)/(tir.d_x + tir.v_n);
    y_e_dot     = (-tir.c_y*y_e)/(tir.d_y + tir.v_n);
    psi_e_dot   = (-tir.c_psi*psi_e)/(tir.d_psi + tir.v_n);
    return
end

% Circular cross section
r_C      = tir.b_0/2; % Radius of circular cross section
dz_circ  = r_C - (r_st - tir.r_0 + r_C)*cos(gamma);
w_circ   = 2*sqrt(r_C^2 - (r_C - dz_circ)^2);
y_Q_circ = (r_st - tir.r_0 + r_C)*sin(gamma);

% Blended deflection for rounded cross section
dz  = (1-tir.R_N)*dz_rect  + tir.R_N*dz_circ;
w   = (1-tir.R_N)*w_rect   + tir.R_N*w_circ;
y_Q = (1-tir.R_N)*y_Q_rect + tir.R_N*y_Q_circ;

% Contact patch shape
L   = 2*sqrt(tir.r_0*dz); % Contact patch length
R_B = 1/6*(L + w);        % Bore radius

%% Normal force
F_z     = dz*tir.c_z - v_z*tir.d_z; % Normal force
F_zUnlim= F_z; % Normal force that stays unlimited

if F_z*mu_n <= 0
    % No normal force or no friction
    s_x = 0; s_y = 0; s_b = 0;
    F_x = 0; F_y = 0; F_z = 0;
    T_x = 0; T_y = 0; T_z = 0;
    x_e_dot     = (-tir.c_x*x_e)/(tir.d_x + tir.v_n);
    y_e_dot     = (-tir.c_y*y_e)/(tir.d_y + tir.v_n);
    psi_e_dot   = (-tir.c_psi*psi_e)/(tir.d_psi + tir.v_n);
    return
elseif F_z > tir.F_zMax
    F_z = tir.F_zMax;
    disp(['Warning: Tire normal load too high (' num2str(F_zUnlim) 'N)!']);
end

%% Slip
omega_c = omega_y*sin(gamma);           % Camber rotation
omega_n = omega_c + omega_z*cos(gamma); % Bore rotation

s_x =  (v_T - v_x)      /v_TStar; % Longitudinal slip
s_y =       - v_y       /v_TStar; % Lateral slip
s_c =       -L/2*omega_c/v_TStar; % Camber slip
s_b =       -R_B*omega_n/v_TStar; % Bore slip

%% Force parameters
if sign(s_x) == sign(v_T)
    dF_x0 = forceParInterp(tir.dF_x01,tir.dF_x02,tir.F_zN,F_z);
    s_xM  = slipParInterp(tir.s_xMA1,tir.s_xMA2,tir.F_zN,F_z)  *mu_n;
    F_xM  = forceParInterp(tir.F_xMA1,tir.F_xMA2,tir.F_zN,F_z) *mu_n;
    s_xS  = slipParInterp(tir.s_xSA1,tir.s_xSA2,tir.F_zN,F_z)  *mu_n;
    F_xS  = forceParInterp(tir.F_xSA1,tir.F_xSA2,tir.F_zN,F_z) *mu_n;
else
    dF_x0 = forceParInterp(tir.dF_x01,tir.dF_x02,tir.F_zN,F_z);
    s_xM  = slipParInterp(tir.s_xMB1,tir.s_xMB2,tir.F_zN,F_z)  *mu_n;
    F_xM  = forceParInterp(tir.F_xMB1,tir.F_xMB2,tir.F_zN,F_z) *mu_n;
    s_xS  = slipParInterp(tir.s_xSB1,tir.s_xSB2,tir.F_zN,F_z)  *mu_n;
    F_xS  = forceParInterp(tir.F_xSB1,tir.F_xSB2,tir.F_zN,F_z) *mu_n;
end

% Fix extrapolation errors
if F_xS > F_xM
    s_xS = s_xM;
    F_xS = F_xM;
end
if s_xS < s_xM
    s_xS = s_xM;
    F_xS = F_xM;
end

dF_y0 = forceParInterp(tir.dF_y01,tir.dF_y02,tir.F_zN,F_z);
s_yM  = slipParInterp(tir.s_yM1,tir.s_yM2,tir.F_zN,F_z)  *mu_n;
F_yM  = forceParInterp(tir.F_yM1,tir.F_yM2,tir.F_zN,F_z) *mu_n;
s_yS  = slipParInterp(tir.s_yS1,tir.s_yS2,tir.F_zN,F_z)  *mu_n;
F_yS  = forceParInterp(tir.F_yS1,tir.F_yS2,tir.F_zN,F_z) *mu_n;

% Fix extrapolation errors
if F_yS > F_yM
    s_yS = s_yM;
    F_yS = F_yM;
end
if s_yS < s_yM
    s_yS = s_yM;
    F_yS = F_yM;
end

%% Normalized slip
s_x_hat = s_xM/(s_xM+s_yM) + (F_xM/dF_x0)/(F_xM/dF_x0+F_yM/dF_y0);
s_y_hat = s_yM/(s_xM+s_yM) + (F_yM/dF_y0)/(F_xM/dF_x0+F_yM/dF_y0);

s_xN = s_x/s_x_hat;
s_yN = s_y/s_y_hat;
s_N  = sqrt(s_xN^2 + s_yN^2);

if s_N > 0
    cosphi = abs(s_xN)/s_N;
    sinphi = abs(s_yN)/s_N;
else
    cosphi = 0;
    sinphi = 1;
end

%% Horizontal forces
% Parameters for combined force calculation
dF_0 = sqrt((dF_x0*s_x_hat*cosphi)^2 + (dF_y0*s_y_hat*sinphi)^2); % Slip stiffness
F_M  = sqrt((F_xM*cosphi)^2 + (F_yM*sinphi)^2);                   % Max. force
s_M  = sqrt((s_xM/s_x_hat*cosphi)^2 + (s_yM/s_y_hat*sinphi)^2);   % Slip for max. force
F_S  = sqrt((F_xS*cosphi)^2 + (F_yS*sinphi)^2);                   % Sliding force
s_S  = sqrt((s_xS/s_x_hat*cosphi)^2 + (s_yS/s_y_hat*sinphi)^2);   % Slip for sliding force

if dF_0 < 2*F_M/s_M
    dF_0 = 2*F_M/s_M;
    disp('Warning: Tire slip stiffness value dF_0 too low!');
end

% Generalized slip
s_G = sqrt(s_xN^2 + s_yN^2 + s_b^2);

% Global tire force derivative
if s_G > s_S
    % Full sliding
    f_G = F_S/s_G;

else
    % Adhesion
    if s_G < s_M
        s_n = s_G/s_M;
        D   = 1 + s_n*(s_n + dF_0*s_M/F_M - 2);
        f_G = dF_0/D;

    % Transition from adhesion to sliding
    else
        a = (F_M/s_M)^2/(dF_0*s_M);
        s_star = s_M + (F_M-F_S)/(a*(s_S-s_M));

        % Case: s_M <= s <= s_S
        if s_star < s_S

            % Case: s_M <= s < s_star
            if s_G < s_star
                F = F_M - a*(s_G-s_M)^2;

            % Case: s_star <= s <= s_S
            else
                b = a*(s_star-s_M)/(s_S-s_star);
                F = F_S + b*(s_S-s_G)^2;
            end
        else
            disp('Warning: Tire full sliding slip value s_S too low!');
            sigma = (s_G-s_M)/(s_S-s_M);
            F = F_M - (F_M-F_S)*sigma^2*(3-2*sigma);
        end

        f_G = F/s_G;
    end
end

% Steady state camber force
F_yc = 1/3*f_G*s_c*tir.eta_c;

if dynamic
    % Tire deflection derivatives
    v_TxStar = v_TStar*s_x_hat;
    v_TyStar = v_TStar*s_y_hat;

    x_e_dot = (-v_TxStar*tir.c_x*x_e - f_G*(v_x-v_T)) ...
        / (v_TxStar*tir.d_x + f_G);
    y_e_dot = (-v_TyStar*tir.c_y*y_e - f_G*(v_y+L/6*omega_c*tir.eta_c)) ...
        / (v_TyStar*tir.d_y + f_G);

    % Dynamic tire forces in x and y
    F_x = tir.c_x*x_e + tir.d_x*x_e_dot;
    F_y = tir.c_y*y_e + tir.d_y*y_e_dot;
    F_yStar = F_y - F_yc;
else
    % No dynamics
    x_e_dot = 0;
    y_e_dot = 0;
    psi_e_dot = 0;

    % Steady state tire forces in x and y
    F_x     = f_G*s_xN;
    F_yStar = f_G*s_yN;
    F_y     = F_yStar + F_yc;
end

%% Torques
% Overturning torque T_x
if dynamic
    T_x = (y_e+y_Q)*F_z;
else
    dy  = F_y/tir.c_y;
    T_x = (dy+y_Q)*F_z;
end

% Rolling resistance torque T_y
T_y = -F_z*tir.a_r*r_0 *v_T/v_TStar;

% Aligning torque T_z
nu_0 = slipParInterp(tir.nu_01,tir.nu_02,tir.F_zN,F_z);
s_y0 = slipParInterp(tir.s_y01,tir.s_y02,tir.F_zN,F_z) *mu_n;
s_yE = slipParInterp(tir.s_yE1,tir.s_yE2,tir.F_zN,F_z) *mu_n;

s_yAbs = abs(s_y);

% Normalized pneumatic trail
if s_yAbs < s_yE
    a  = -(2*s_y0^2 + (s_y0+s_yE)^2);
    b  =  (2*(s_y0+s_yE)^2) / s_yE;
    c  = -(2*s_y0 + s_yE)   / s_yE;
    nu = nu_0*(1 + (s_yAbs/(s_y0*s_yE))^2 * (a + s_yAbs*(b + s_yAbs*c)))*sinphi;
else
    nu = 0;
end

% Self-aligning torque
T_S = -nu*L*F_yStar;

if dynamic
    % Torsional deflection derivative
    psi_e_dot = (-v_TStar*tir.c_psi*psi_e - R_B^2*f_G*omega_n) ...
        / (v_TStar*tir.d_psi + R_B^2*f_G);

    % Dynamic bore torque
    T_B = tir.c_psi*psi_e + tir.d_psi*psi_e_dot;
else
    % Steady state bore torque
    T_B = f_G*R_B*s_b*tir.eta_b;
end

% T_z
T_z = T_S + T_B;


%% Parameter interpolation functions
    function X = slipParInterp(X1,X2,Z1,Z)
        X = X1 + (X2-X1)*(Z/Z1-1);
    end

    function Y = forceParInterp(Y1,Y2,Z1,Z)
        Y = Z/Z1 *( 2*Y1 - 0.5*Y2 - (Y1 - 0.5*Y2)*Z/Z1 );
    end

end