function [T_MCmdFL,T_MCmdFR,T_MCmdRL,T_MCmdRR] = f_eMotTorqueCtrl(veh,tirFL,tirFR,tirRL,tirRR,x,throttle)
%%      Electric motor torque control      %%
% ----------------------------------------- %
% Version: V1.1 - 2024.12.26                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Add custom functions here.          %
% Input:                                    %
%   veh     -   struct containing vehicle   %
%               parameters                  %
%   tirXX   -   struct containing tire      %
%               parameters for one tire     %
%               where XX is FL/FR/RL/RR     %
%   x       -   state vector                %
%   throttle -  normalized throttle         %
% Output:                                   %
%   T_MXX   -   Motor torque                %
%               where XX is FL/FR/RL/RR     %
% ----------------------------------------- %
TCenabled = true;

if TCenabled == false
    %% Torques without slip controller
    T_MCmdFL = throttle*veh.T_MMax;
    T_MCmdFR = throttle*veh.T_MMax;
    T_MCmdRL = throttle*veh.T_MMax;
    T_MCmdRR = throttle*veh.T_MMax;

else
    %% Example of a simple slip controller
    % Useful signals
    z = x(15:28);
    v_x=z(1); v_y=z(2); v_z=z(3); omega_x=z(4); omega_y=z(5); omega_z=z(6); %#ok<NASGU>
    omega_FL=z(11); omega_FR=z(12); omega_RL=z(13); omega_RR=z(14);

    % Motor commands
    T_MCmdFL = slipControl(veh,tirFL,omega_FL,v_x,throttle);
    T_MCmdFR = slipControl(veh,tirFR,omega_FR,v_x,throttle);
    T_MCmdRL = slipControl(veh,tirRL,omega_RL,v_x,throttle);
    T_MCmdRR = slipControl(veh,tirRR,omega_RR,v_x,throttle);
end

% Simple slip controller function
    function T_MCmd = slipControl(veh,tir,omega_y,v_x,throttle)
        s_xTar  = 0.15; % Target slip
        k_p     = 100;  % Controller gain

        v_T = omega_y*tir.r_0;          % Transport velocity
        v_TStar = abs(v_T) + tir.v_n;   % Absolute transport velocity + v_n
        s_x =  (v_T - v_x) /v_TStar;    % Longitudinal slip
        e_sx = min([0 s_xTar - s_x]);   % Slip error (only negative)

        T_MCmd = throttle*veh.T_MMax + k_p*e_sx; % Motor torque command

        if T_MCmd < 0
            T_MCmd = 0;
        end
    end
end