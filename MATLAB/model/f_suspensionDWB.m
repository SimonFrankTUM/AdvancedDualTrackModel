function [A_VW,r_VWV,delta,dvdphi,domdphi,dvdu,domdu] = f_suspensionDWB(sus,phi,u,t)
%%      Double Wishbone Suspension         %%
% ----------------------------------------- %
% Version: V1.3 - 2024.12.26                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info:                                     %
% Input:                                    %
%   phi     -   rotation angle of lower     %
%               wishbone                    %
%   u       -   steering rack travel        %
%   sus     -   struct containing           %
%               suspension parameters       %
%   t       -   current simulation time     %
% Output:                                   %
%   A_VW    -   rotation matrix from wheel  %
%               to vehicle frame            %
%   r_VWV   -   distance vector from CoG    %
%               to wheel center in vehicle  %
%               frame                       %
%   dvdphi  -   dv_U / dphi_dot             %
%   domdphi -   domega_U / dphi_dot         %
%   dvdu    -   dv_U / du_dot               %
%   domdu   -   domega_U / du_dot           %
% ----------------------------------------- %

%% Input check
if phi > sus.phi_max
    error([num2str(t) 's: Lower wishbone angle too high (' num2str(phi*180/pi) '°).']);
end

%% Upright position and orientation
% Rotation of lower wishbone
r_ABV = sus.r_VBk - sus.r_VAk;
e_ABV = r_ABV/norm(r_ABV);
e_ABV_tilde = f_crossprod(e_ABV);
e_ABV2 = e_ABV*e_ABV';
A_phi  = e_ABV2 + (eye(3,3)-e_ABV2)*cos(phi) + e_ABV_tilde*sin(phi);

% Rotation of upper wishbone
r_CFk = sus.r_VFk - sus.r_VCk;
r_DFk = sus.r_VFk - sus.r_VDk;
r_ACV = A_phi*(sus.r_VCk-sus.r_VAk);
r_CDV = sus.r_VDk - (sus.r_VAk+r_ACV);
r_DEV = sus.r_VEk - sus.r_VDk;
e_DEV = r_DEV/norm(r_DEV);
e_DEV2= e_DEV*e_DEV';
e_DEV_tilde = f_crossprod(e_DEV);
a = r_CDV'*(eye(3,3)-e_DEV2)*r_DFk;
b = r_CDV'*cross(e_DEV,r_DFk);
c =-r_CDV'*e_DEV2*r_DFk - 0.5*(r_DFk'*r_DFk+r_CDV'*r_CDV-r_CFk'*r_CFk);
psi = solveForAngle(a,b,c);
A_psi = e_DEV2 + (eye(3,3)-e_DEV2)*cos(psi) + e_DEV_tilde*sin(psi);

% Rotation of upright around X & Y axis
r_VCV = sus.r_VAk + r_ACV;
r_DFV = A_psi*(sus.r_VFk-sus.r_VDk);
r_VFV = sus.r_VDk + r_DFV;
r_CFV = r_VFV - r_VCV;

% Angle alpha around X axis
alpha   = solveForAngle(r_CFV(2),r_CFV(3),r_CFk(2));
A_alpha = f_xrot(alpha);

% Angle beta around Y axis
beta   = solveForAngle(r_CFk(1),r_CFk(3),r_CFV(1));
A_beta = f_yrot(beta);

% Steering angle delta
r_VRV = sus.r_VRk + [0; u; 0];
r_RCV = r_VCV - r_VRV;
r_RCH_transp = r_RCV'*A_alpha*A_beta;
r_RQk = sus.r_VQk - sus.r_VRk;
r_CQk = sus.r_VQk - sus.r_VCk;
e_CFW = r_CFk/norm(r_CFk);
e_CFW2= e_CFW*e_CFW';
e_CFW_tilde = f_crossprod(e_CFW);
a = r_RCH_transp*(eye(3,3)-e_CFW2)*r_CQk;
b = r_RCH_transp*cross(e_CFW,r_CQk);
c = -(r_RCH_transp*e_CFW2*r_CQk+0.5*(r_RCV'*r_RCV+r_CQk'*r_CQk-r_RQk'*r_RQk));

delta = solveForAngle(a,b,c);
A_delta = e_CFW2 + (eye(3,3)-e_CFW2)*cos(delta) + e_CFW_tilde*sin(delta);

% Upright orientation and position
A_VW  = A_alpha*A_beta*A_delta*f_zrot(sus.delta_0)*f_xrot(sus.gamma_0); % X, Y, steering, toe, inclination angle
delta = delta + sus.delta_0;
r_CWV = A_VW*(sus.r_VWk-sus.r_VCk);
r_VWV = r_VCV + r_CWV;

%% Partial velocities
% Partial derivatives dpsi/dphi, dalpha/dphi, dbeta/dphi
e_XV = [1;0;0];
e_YV = [0;1;0];
e_YalphaV = A_alpha*e_YV;
a = [cross(e_DEV,r_DFV)  -cross(e_XV,r_CFV)  -cross(e_YalphaV,r_CFV)];
c = a \ cross(e_ABV,r_ACV);
dpsidphi    = c(1); %#ok<NASGU>
dalphadphi  = c(2);
dbetadphi   = c(3);

% Partial derivatives ddelta/dphi, ddelta/du
e_CFV = r_CFV/norm(r_CFV);
r_CQV = A_VW*r_CQk;
r_VQV = r_VCV + r_CQV;
r_RQV = r_VQV - r_VRV;
a = e_XV*dalphadphi + e_YalphaV*dbetadphi;
b = cross(e_ABV,r_ACV) + cross(a,r_CQV);
c = r_RQV'*cross(e_CFV,r_CQV);
ddeltadphi =-r_RQV'*b/c;
ddeltadu   = r_RQV'*e_YV/c;

% Partial angular and partial velocities of upright
domdphi = e_XV*dalphadphi + e_YalphaV*dbetadphi + e_CFV*ddeltadphi;
dvdphi  = cross(e_ABV,r_ACV) + cross(domdphi,r_CWV);
domdu   = e_CFV*ddeltadu;
dvdu    = cross(domdu,r_CWV);

%% Additional functions
    function ang = solveForAngle(a,b,c)
        % solve a*cos(ang) + b*sin(ang) = c for angle ang
        if b < 0
            ang = asin(-c/sqrt(a^2+b^2)) - atan2(-a,-b);
        else
            ang = asin( c/sqrt(a^2+b^2)) - atan2( a, b);
        end
    end

end