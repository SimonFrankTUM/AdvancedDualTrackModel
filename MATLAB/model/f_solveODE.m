function [TTout,opts] = f_solveODE(input,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts)
%%          Vehicle Model Solver           %%
% ----------------------------------------- %
% Version: V1.7 - 2025.05.21                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Numerical integration               %
%       with constant step size             %
% Input:                                    %
%   input   -   struct of model input       %
%               signals                     %
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
%   TTout   -   timetable of output signals %
%   opts    -   struct of additional model  %
%               options                     %
% ----------------------------------------- %
warning('off','backtrace');

%% Initialization
tStep       = 1/opts.simFreq;
nSubSteps   = opts.simFreq/opts.dataFreq;
tData       = (0:1/opts.dataFreq:opts.tEnd)';
nData       = length(tData);
xData       = NaN(nData,opts.nStates);
output      = NaN(nData,opts.nOutputs);
wbmsg       = ['Simulating using ' opts.solver ' solver'];

if nSubSteps ~= round(nSubSteps)
    error('Simulation frequency must be multiple of data frequency.');
end

%% Initial states and outputs
t = 0;
x = opts.x_0;
[x(1),x(2),x(3),x(4),x(5),x(7),x(8),x(9),x(10)] = f_calcInitState(veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);
[~,out] = f_vehicleModel(input(1),t,x,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);

%% Integration
% Start timer and create waitbar
tic;
wb = waitbar(0,wbmsg,'Name','Simulation Progress'); wb.Pointer = 'watch'; waitbar(0,wb);

% Use selected solver
switch opts.solver

    case 'Euler'
        for step = 1:nData
            xData(step,:)   = x';
            output(step,:)  = out';

            try
                if step < nData
                    for subStep = 1:nSubSteps
                        [x_dot,out] = f_vehicleModel( ...
                            input((step-1)*nSubSteps+subStep),t,x,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);
                        t = t + tStep;
                        x = x + tStep.*x_dot;
                    end
                end

                uistep = step*2E-2;
                if uistep == floor(uistep)
                    waitbar(step/nData,wb);
                end
                
            catch ME
                if ~strncmp(ME.message,'The second argument',19)
                    warning(ME.message); % Not waitbar related errors
                end
                disp(['Simulation stopped after ' num2str(t) 's.']);
                break
            end
        end
        close hidden all; % Close waitbar

    case 'Heun'
        for step = 1:nData
            xData(step,:)   = x';
            output(step,:)  = out';

            try
                if step < nData
                    for subStep = 1:nSubSteps
                        [x_dot_current,out] = f_vehicleModel( ...
                            input((step-1)*nSubSteps+subStep),t,x,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);
                        t = t + tStep;
                        [x_dot_next,~]      = f_vehicleModel( ...
                            input((step-1)*nSubSteps+subStep),t,x+tStep.*x_dot_current,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);
                        x_dot = 0.5.*(x_dot_current+x_dot_next);
                        x = x + tStep.*x_dot;
                    end
                end

                uistep = step*2E-2;
                if uistep == floor(uistep)
                    waitbar(step/nData,wb);
                end

            catch ME
                if ~strncmp(ME.message,'The second argument',19)
                    warning(ME.message); % Not waitbar related errors
                end
                disp(['Simulation stopped after ' num2str(t) 's.']);
                break
            end
        end
        close hidden all; % Close waitbar

    case 'Runge-Kutta'
        for step = 1:nData
            xData(step,:)   = x';
            output(step,:)  = out';

            try
                if step < nData
                    for subStep = 1:nSubSteps
                        [k1,out]= f_vehicleModel( ...
                            input((step-1)*nSubSteps+subStep),t,x,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);
                        t = t+0.5*tStep;
                        [k2,~]  = f_vehicleModel( ...
                            input((step-1)*nSubSteps+subStep),t,x+0.5.*tStep.*k1,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);
                        [k3,~]  = f_vehicleModel( ...
                            input((step-1)*nSubSteps+subStep),t,x+0.5.*tStep.*k2,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);
                        t = t+0.5*tStep;
                        [k4,~]  = f_vehicleModel( ...
                            input((step-1)*nSubSteps+subStep),t,x+tStep.*k3,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);
                        x_dot   = (k1 + 2.*k2 + 2.*k3 + k4)./6;
                        x = x + tStep.*x_dot;
                    end
                end

                uistep = step*2E-2;
                if uistep == floor(uistep)
                    waitbar(step/nData,wb);
                end

            catch ME
                if ~strncmp(ME.message,'The second argument',19)
                    warning(ME.message); % Not waitbar related errors
                end
                disp(['Simulation stopped after ' num2str(t) 's.']);
                break
            end
        end
        close hidden all; % Close waitbar

    otherwise
        close hidden all; % Close waitbar
        error('Chosen solver does not exit.');
end

%% Store simulation results in timetable
opts.tEnd = t;
TTout = splitvars(timetable(seconds(tData),[xData output]));
TTout.Properties.VariableNames = {'x_V','y_V','z_V','phi','theta','psi', ...
    'phi_FL','phi_FR','phi_RL','phi_RR','rho_FL','rho_FR','rho_RL','rho_RR', ...
    'v_x','v_y','v_z','omega_x','omega_y','omega_z', ...
    'phi_FL_dot','phi_FR_dot','phi_RL_dot','phi_RR_dot', ...
    'omega_FL','omega_FR','omega_RL','omega_RR', ...
    'x_eFL','y_eFL','psi_eFL', ...
    'x_eFR','y_eFR','psi_eFR', ...
    'x_eRL','y_eRL','psi_eRL', ...
    'x_eRR','y_eRR','psi_eRR', ...
    'a_x','a_y','a_z', ...
    'F_xFL','F_yFL','F_zFL', ...
    'F_xFR','F_yFR','F_zFR', ...
    'F_xRL','F_yRL','F_zRL', ...
    'F_xRR','F_yRR','F_zRR', ...
    's_xFL','s_yFL','s_cFL','s_bFL', ...
    's_xFR','s_yFR','s_cFR','s_bFR', ...
    's_xRL','s_yRL','s_cRL','s_bRL', ...
    's_xRR','s_yRR','s_cRR','s_bRR', ...
    'F_ycFL','F_ycFR','F_ycRL','F_ycRR', ...
    'dF_x0FL','dF_x0FR','dF_x0RL','dF_x0RR', ...
    'F_xMFL','F_xMFR','F_xMRL','F_xMRR', ...
    'dF_y0FL','dF_y0FR','dF_y0RL','dF_y0RR', ...
    'F_yMFL','F_yMFR','F_yMRL','F_yMRR', ...
    'r_stFL','r_stFR','r_stRL','r_stRR', ...
    'r_dynFL','r_dynFR','r_dynRL','r_dynRR', ...
    'z_FL','z_FR','z_RL','z_RR', ...
    'z_FL_dot','z_FR_dot','z_RL_dot','z_RR_dot', ...
    'F_zPRFL','F_zPRFR','F_zPRRL','F_zPRRR', ...
    'delta_FL','delta_FR','delta_RL','delta_RR', ...
    'gamma_FL','gamma_FR','gamma_RL','gamma_RR', ...
    'T_BrakeFL','T_BrakeFR','T_BrakeRL','T_BrakeRR', ...
    'T_DriveFL','T_DriveFR','T_DriveRL','T_DriveRR', ...
    'omega_Mot','T_Mot','F_xAero','F_yAero','F_zAero','T_Steer'};

% Stop timer and beep
toc;
beep;

end