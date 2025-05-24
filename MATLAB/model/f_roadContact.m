function [v_0QQ,A_0Q,gamma,r_st,dz,w,r_MQ0,mu_R] = f_roadContact(road,tir,r_0M0,v_0M0,omega_0U0,e_yR0,opts,t)
%%          Road Contact Model             %%
% ----------------------------------------- %
% Version: V1.6 - 2024.06.21                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
%   road    -   struct of road              %
%               x, y, Z, and Mu data        %
%   tir     -   struct containing tire      %
%               parameters for one tire     %
%   r_0M0   -   distance vector from origin %
%               to wheel center M in origin %
%               frame                       %
%   v_0M0   -   velocity vector between     %
%               origin and wheel center M   %
%               in origin frame             %
%   omega_0Ustar0 - angular velocity vector %
%               of upright in relation to   %
%               origin in origin frame      %
%               adjusted for wheel bearings %
%   e_yR0   -   wheel rotation axis in      %
%               origin frame                %
%   t       -   current simulation time     %
% Output:                                   %
%   v_0QQ   -   velocity vector between     %
%               origin and contact point Q  %
%               in Q frame                  %
%   A_0Q    -   rotation matrix from Q to   %
%               origin frame                %
%   gamma   -   inclination angle           %
%   r_st    -   static tire radius          %
%   dz      -   vertical tire deflection    %
%   w       -   width of tire contact patch %
%   r_MQ0   -   distance vector from wheel  %
%               center M to contact point Q %
%               in origin frame             %
%   mu_R    -   local friction coefficient  %
% ----------------------------------------- %

%% Road contact
e_n00   = [0; 0; 1];    % Global road normal
b2      = 0.5*tir.b_0;  % Half tire width b/2

% First guess contact point
e_xStar = cross(e_yR0,e_n00);   % Kind of forward
e_xStar = e_xStar/sqrt(e_xStar(1)^2 + e_xStar(2)^2 + e_xStar(3)^2); % Adjust length to 1
e_zStar = cross(e_xStar,e_yR0); % Kind of up

r_0PStar = r_0M0 - (tir.r_0-b2)*e_zStar - b2*e_n00; % Wheel center -> contact point guess P*

if opts.use3DRoad % 3D road
    % Put P* on road surface
    r_0PStar(3) = f_roadHeight(road, r_0PStar(1), r_0PStar(2));

    % Sample offsets
    dx = 0.1*tir.r_0;
    dy = 0.3*tir.b_0;

    % Road sample points
    r_0RStar1 = r_0PStar + dx*e_xStar;
    r_0RStar2 = r_0PStar - dx*e_xStar;
    r_0RStar3 = r_0PStar + dy*e_yR0;
    r_0RStar4 = r_0PStar - dy*e_yR0;

    % Heights of sample points
    z_0R1 = f_roadHeight(road, r_0RStar1(1), r_0RStar1(2),t);
    z_0R2 = f_roadHeight(road, r_0RStar2(1), r_0RStar2(2),t);
    z_0R3 = f_roadHeight(road, r_0RStar3(1), r_0RStar3(2),t);
    z_0R4 = f_roadHeight(road, r_0RStar4(1), r_0RStar4(2),t);

    % Sample points with height
    r_0R1 = [r_0RStar1(1); r_0RStar1(2); z_0R1];
    r_0R2 = [r_0RStar2(1); r_0RStar2(2); z_0R2];
    r_0R3 = [r_0RStar3(1); r_0RStar3(2); z_0R3];
    r_0R4 = [r_0RStar4(1); r_0RStar4(2); z_0R4];
    r_R2R1 = r_0R1 - r_0R2;
    r_R4R3 = r_0R3 - r_0R4;

    % Road normal
    e_n = cross(r_R2R1,r_R4R3);
    e_n = e_n/sqrt(e_n(1)^2 + e_n(2)^2 + e_n(3)^2); % Adjust length to 1

    % Geometric contact point P
    r_0R0 = 0.25*(r_0R1 + r_0R2 + r_0R3 + r_0R4);
    r_MR0 = r_0R0 - r_0M0;

else % Flat road
    % Road normal on flat road
    e_n = e_n00;

    % Road contact point on flat road
    r_0R0 = r_0PStar.*[1;1;0];
    r_MR0 = r_0R0 - r_0M0;

end

% Unit vectors of Q frame
e_xQ = cross(e_yR0,e_n);    % Forward on road surface
e_xQ = e_xQ/sqrt(e_xQ(1)^2 + e_xQ(2)^2 + e_xQ(3)^2); % Adjust length to 1
e_yQ = cross(e_n,e_xQ);     % Left on road surface
e_zR = cross(e_xQ,e_yR0);   % Up on wheel rot. axis
A_0Q = [e_xQ e_yQ e_n];     % Rot. matrix from Q to 0

% Inclination angle ("camber")
gamma = asin(e_yR0'*e_n);

% Static tire radius
r_st  = -(e_n'*r_MR0)/(e_n'*e_zR);

if r_st > tir.r_0
    r_st = tir.r_0;
end

r_0P0 = r_0M0 - r_st*e_zR;

%% Tire deflection
% Rectangular cross section
r_stL = r_st + b2 * tan(gamma);
r_stR = r_st - b2 * tan(gamma);

if r_stL < tir.r_0 && r_stR < tir.r_0
    % Full contact
    dz_rect  =  tir.r_0 - r_st;
    w_rect   =  tir.b_0/cos(gamma);
    y_Q_rect = -tir.b_0^2*tan(gamma) / (12*dz_rect*cos(gamma));

elseif r_stL >= tir.r_0 && r_stR < tir.r_0 && gamma ~= 0
    % Left edge lifts off road
    dz_rect  = (tir.r_0-r_stR)^2 / (2*tir.b_0*tan(abs(gamma)));
    w_rect   = (tir.r_0-r_stR) / tan(abs(gamma));
    y_Q_rect = -sign(gamma)*(b2/cos(gamma) - w_rect/3);

elseif r_stL < tir.r_0 && r_stR >= tir.r_0 && gamma ~= 0
    % Right edge lifts off road
    dz_rect  = (tir.r_0-r_stL)^2 / (2*tir.b_0*tan(abs(gamma)));
    w_rect   = (tir.r_0-r_stL) / tan(abs(gamma));
    y_Q_rect = -sign(gamma)*(b2/cos(gamma) - w_rect/3);

else
    % No contact
    dz_rect  = 0;
    w_rect   = tir.b_0;
    y_Q_rect = 0;
    gamma    = 0;
end

% Circular cross section
r_C      = b2; % Radius of circular cross section
dz_circ  = r_C - (r_st - (tir.r_0 - r_C))*cos(gamma);
w_circ   = 2*sqrt(r_C^2 - (r_C - dz_circ)^2);
y_Q_circ = -(r_st - (tir.r_0 - r_C))*sin(gamma);

% Blended deflection for rounded cross section
dz  = (1-tir.R_N)*dz_rect  + tir.R_N*dz_circ;
w   = (1-tir.R_N)*w_rect   + tir.R_N*w_circ;
y_Q = (1-tir.R_N)*y_Q_rect + tir.R_N*y_Q_circ;

%% Contact velocity
% Static contact point Q
r_0Q0 = r_0P0 + y_Q*e_yQ;
r_MQ0 = r_0Q0 - r_0M0;

% Upright angular velocity with wheel bearing taken into account
omega_0U0star = omega_0U0 - e_yR0'*omega_0U0*e_yR0;

% Contact point velocity
v_0QQ = A_0Q'*(v_0M0 + cross(omega_0U0star,r_MQ0));

if dz == 0
    v_0QQ(3) = 0;
end

%% Local friction coefficient
if opts.use3DRoad % Mu from road file
    mu_R = f_roadMu(road,r_0Q0(1),r_0Q0(2));
else
    mu_R = 1.0;
end

end