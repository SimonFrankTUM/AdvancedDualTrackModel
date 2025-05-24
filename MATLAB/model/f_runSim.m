function [TTin,TTout,road,opts] = f_runSim(opts)
%%              Run Simulation             %%
% ----------------------------------------- %
% Version: V1.0 - 2025.05.21                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Perform a simulation run            %
% Input:                                    %
%   opts    -   struct of additional model  %
%               options                     %
% Output:                                   %
%   TTin    -   timetable of input signals  %
%   TTout   -   timetable of output signals %
%   road    -   struct of road              %
%               x, y, Z, and Mu data        %
%   opts    -   struct of additional model  %
%               options                     %
% ----------------------------------------- %

% Number of states and outputs used to create arrays with correct sizes
opts.nStates    = 40;
opts.nOutputs   = 93;

% Interactive parameter loading
veh   = f_vehicleParameters();
tirFL = f_tireParameters(veh.tires{1});
tirFR = f_tireParameters(veh.tires{2});
tirRL = f_tireParameters(veh.tires{3});
tirRR = f_tireParameters(veh.tires{4});
env   = f_environmentParameters(opts.envConditions);
[TTin,input,road,opts] = f_loadInput(opts);
switch opts.tireModelName
    case 'TMeasy'
        opts.tireModel = 2;
    case 'linear'
        opts.tireModel = 1;
        if opts.tireDyn
            error('Linear tire model has no dynamic mode.');
        end
    otherwise
        error('Chosen tire model does not exit.');
end

% Solve simulation ODEs
[TTout,opts] = f_solveODE(input,veh,tirFL,tirFR,tirRL,tirRR,env,road,opts);

% Cleanup
clearvars -except envConditions road opts TTin TTout veh;

end