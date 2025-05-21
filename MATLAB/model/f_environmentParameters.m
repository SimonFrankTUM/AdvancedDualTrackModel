function env = f_environmentParameters(envConditions)
%%          Environment Parameters         %%
% ----------------------------------------- %
% Version: V1.0 - 2024.01.02                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
%   envConditions - name of chosen          %
%               environment parameter set   %
%               as char                     %
% Output:                                   %
%   env     -   struct containing           %
%               environment parameters      %
% ----------------------------------------- %

switch envConditions
    
    case 'default_mu0.5'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 0.5;      % -

    case 'default_mu0.6'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 0.6;      % -

    case 'default_mu0.7'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 0.7;      % -

    case 'default_mu0.8'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 0.8;      % -

    case 'default_mu0.9'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 0.9;      % -

    case 'default_mu1.0'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 1.0;      % -

    case 'default_mu1.1'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 1.1;      % -

    case 'default_mu1.2'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 1.2;      % -

    case 'default_mu1.3'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 1.3;      % -

    case 'default_mu1.4'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 1.4;      % -

    case 'default_mu1.5'
        g       = 9.80665;  % m/s^2
        T_Amb   = 20;       % °C
        p_Amb   = 1013.25;  % hPa
        mu_Road = 1.5;      % -

    otherwise
        error('Unknown environment conditions selected.');
end

env.g       = g; % m/s^2
env.T_Amb   = T_Amb; % °C
env.rho_Air = p_Amb*1e2 / (287.058 * (T_Amb + 273.15)); % kg/m^3
env.mu_N    = mu_Road; % -

end